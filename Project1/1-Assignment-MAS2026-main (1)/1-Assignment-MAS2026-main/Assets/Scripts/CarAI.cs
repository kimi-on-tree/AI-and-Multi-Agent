using System;
using System.Collections.Generic;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

[RequireComponent(typeof(CarController), typeof(Rigidbody))]
public class CarAI : Agent
{
    public CarController car;
    Rigidbody rb;

    [Header("Hybrid A*")]
    public float cellSize = 0.5f;
    public int thetaBins = 96;
    public float stepSize = 0.75f;
    public float maxSteerDeg = 25f;
    public int steerSamples = 9;
    public bool allowReverse = true;
    public float wheelBase = 2.6f;
    public int maxIters = 10_000_000;

    [Header("Collision check")]
    public LayerMask obstacleMask;
    public float inflateR = 0.3f;              // main sphere radius
    public float checkY = 0.1f;                // height offset
    public float probeFwd = 1.2f, probeSide = 0.7f;
    public float probeScale = 0.9f;

    [Header("Follower")]
    public float goalTol = 2.5f;
    public float lookStraight = 6f, lookCorner = 2.5f, minLook = 1.5f;
    public float slowStartDeg = 15f, slowFullDeg = 80f, turnLookahead = 18f;
    public float vStraight = 12f, vTurn = 3f, minThrottle = 0.12f;
    public float steerClamp = 0.8f, steerSmooth = 0.15f;

    [Header("Recovery")]
    public bool enableRecovery = true;
    public float minImpact = 0.5f;
    public float reverseTime = 1.2f, reverseThrottle = 0.45f, reverseSteer = 0.65f, cooldown = 0.6f;

    readonly List<Vector3> pathW = new();
    int pathIdx = 0;
    float steerSm = 0f;

    bool recovering = false;
    float recoverUntil = -1f, cooldownUntil = -1f;
    float recoverSteerSign = 1f;
    bool justRecovered = false;

    // planner state
    BoundsInt bounds;
    int W, H;
    Vector3 originLocal, startLocal, goalLocal;

    struct Rec { public float g, f, theta, lastSteer; public int parent; public Vector3 p; }
    readonly Dictionary<int, Rec> recs = new(200000);
    readonly HashSet<int> closed = new(200000);

    class Heap
    {
        struct It { public int k; public float f; }
        readonly List<It> a = new(1024);
        public int Count => a.Count;
        public void Push(int k, float f)
        {
            a.Add(new It { k = k, f = f });
            for (int i = a.Count - 1; i > 0;)
            {
                int p = (i - 1) >> 1;
                if (a[i].f >= a[p].f) break;
                (a[i], a[p]) = (a[p], a[i]); i = p;
            }
        }
        public int Pop(out float f)
        {
            var root = a[0]; f = root.f;
            int last = a.Count - 1;
            a[0] = a[last]; a.RemoveAt(last);
            for (int i = 0, n = a.Count; ;)
            {
                int l = (i << 1) + 1; if (l >= n) break;
                int r = l + 1;
                int s = (r < n && a[r].f < a[l].f) ? r : l;
                if (a[i].f <= a[s].f) break;
                (a[i], a[s]) = (a[s], a[i]); i = s;
            }
            return root.k;
        }
    }

    public override void Initialize()
    {
        if (!car) car = GetComponent<CarController>();
        rb = GetComponent<Rigidbody>();

        var sW = MapManager.GetGlobalStartPosition();
        var gW = MapManager.GetGlobalGoalPosition();

        startLocal = MapManager.transform.InverseTransformPoint(sW);
        goalLocal  = MapManager.transform.InverseTransformPoint(gW);

        bounds = ObstacleMap.localBounds;
        originLocal = new Vector3(bounds.min.x, 0f, bounds.min.z);

        float cs = Mathf.Max(0.01f, cellSize);
        W = Mathf.Max(1, Mathf.CeilToInt(bounds.size.x / cs));
        H = Mathf.Max(1, Mathf.CeilToInt(bounds.size.z / cs));

        BuildPath(sW, gW);
        steerSm = 0f;
    }

    void OnCollisionEnter(Collision c)
    {
        if (!enableRecovery || recovering) return;
        if (Time.time < cooldownUntil) return;
        if (((1 << c.gameObject.layer) & obstacleMask.value) == 0) return;
        if (c.relativeVelocity.magnitude < minImpact) return;

        var n = (c.contactCount > 0) ? c.contacts[0].normal : -transform.forward;
        var nLocal = transform.InverseTransformDirection(n);
        recoverSteerSign = (nLocal.x > 0f) ? -1f : 1f;

        recovering = true;
        recoverUntil = Time.time + reverseTime;
        cooldownUntil = Time.time + cooldown;
        steerSm = 0f;
    }

    public override void Step()
    {
        if (!car || pathW.Count < 2) return;

        float speed = rb ? rb.linearVelocity.magnitude : 0f;

        // 1) recovery
        if (enableRecovery && recovering)
        {
            if (Time.time >= recoverUntil)
            {
                recovering = false;
                justRecovered = true;
                cooldownUntil = Time.time + cooldown;
                car.Move(0f, 0f, 1f, 0f);
                return;
            }

            float steer = Mathf.Clamp(reverseSteer * recoverSteerSign, -1f, 1f);

            if (speed > 0.6f) { car.Move(steer, 0f, 1f, 0f); return; } // brake to stop
            float rev = -Mathf.Abs(reverseThrottle);
            car.Move(steer, rev, rev, 0f);
            return;
        }

        // 2) stop near goal
        var goal = pathW[^1];
        var toG = goal - transform.position; toG.y = 0f;
        if (toG.magnitude < goalTol) { car.Move(0f, 0f, 1f, 0f); return; }

        // 3) closest point -> path progress
        ClosestOnPath(out int seg, out Vector3 cp, out float cte);
        if (justRecovered) { pathIdx = Mathf.Clamp(seg, 0, pathW.Count - 2); justRecovered = false; }
        else pathIdx = Mathf.Clamp(Mathf.Max(pathIdx, seg), 0, pathW.Count - 2);

        // 4) turn scan + lookahead
        float distCorner;
        float maxTurn = UpcomingTurnDeg(pathIdx, cp, out distCorner);
        float sharpT = Mathf.InverseLerp(slowStartDeg, slowFullDeg, maxTurn);
        float Ld = Mathf.Max(minLook, Mathf.Lerp(lookStraight, lookCorner, sharpT));
        var target = LookaheadPoint(pathIdx, cp, Ld);

        // 5) pure pursuit steering
        var fwd = transform.forward; fwd.y = 0f; fwd = (fwd.sqrMagnitude < 1e-6f) ? Vector3.forward : fwd.normalized;
        var front = transform.position + transform.forward * (wheelBase * 0.5f); front.y = 0f;

        var toT = target - front; toT.y = 0f;
        float steerNorm = 0f;
        float dist = toT.magnitude;
        if (dist > 0.1f)
        {
            toT /= dist;
            float alpha = Vector3.SignedAngle(fwd, toT, Vector3.up) * Mathf.Deg2Rad;
            float steerRad = Mathf.Atan2(2f * wheelBase * Mathf.Sin(alpha), Mathf.Max(0.1f, Ld));
            steerNorm = Mathf.Clamp(steerRad / Mathf.Max(0.01f, maxSteerDeg * Mathf.Deg2Rad), -1f, 1f);
        }

        steerNorm = Mathf.Clamp(steerNorm, -steerClamp, steerClamp);
        steerSm = Mathf.Lerp(steerSm, steerNorm, steerSmooth);

        // 6) throttle/brake (corner-aware)
        bool turnAhead = (maxTurn >= slowStartDeg) && (distCorner <= turnLookahead);

        float targetV = turnAhead ? Mathf.Lerp(vStraight, vTurn, sharpT) : vStraight;
        float throttle = 0f, brake = 0f;

        if (speed < targetV)
        {
            float thr = (targetV - speed) / Mathf.Max(0.1f, targetV);
            throttle = Mathf.Clamp(thr, minThrottle, 1f);
            throttle *= Mathf.Clamp01(1f - 0.8f * Mathf.Abs(steerSm));
        }
        else
        {
            throttle = 0f;
            brake = turnAhead ? 0.5f : 0f;
        }

        car.Move(steerSm, throttle, brake, 0f);

        Debug.DrawLine(transform.position + Vector3.up * 0.25f, target + Vector3.up * 0.25f, Color.magenta, 0.05f);
    }

    // path build 
    void BuildPath(Vector3 startW, Vector3 goalW)
    {
        pathW.Clear(); pathIdx = 0;

        var fwdLocal = MapManager.transform.InverseTransformDirection(transform.forward);
        fwdLocal.y = 0f; if (fwdLocal.sqrMagnitude < 1e-6f) fwdLocal = Vector3.forward;
        fwdLocal.Normalize();
        float startTheta = Mathf.Atan2(fwdLocal.x, fwdLocal.z);

        var local = HybridAStar(startLocal, startTheta, goalLocal);
        if (local == null || local.Count == 0)
        {
            pathW.Add(startW); pathW.Add(goalW);
            return;
        }

        foreach (var r in local) pathW.Add(MapManager.transform.TransformPoint(r.p));
        if (pathW.Count > 0) pathW[0] = startW;
        if (pathW.Count > 1) pathW[^1] = goalW;
    }

    List<Rec> HybridAStar(Vector3 s, float sTheta, Vector3 g)
    {
        recs.Clear(); closed.Clear();
        if (!PoseFree(s, sTheta) || !PoseFree(g, 0f)) return null;

        int sk = Key(s, sTheta);
        var sr = new Rec { g = 0f, f = Dist(s, g), parent = -1, p = s, theta = Wrap(sTheta), lastSteer = 0f };
        recs[sk] = sr;

        var open = new Heap();
        open.Push(sk, sr.f);

        float maxSteer = maxSteerDeg * Mathf.Deg2Rad;
        int S = Mathf.Max(2, steerSamples);

        for (int iter = 0; open.Count > 0 && iter < maxIters; iter++)
        {
            int ck = open.Pop(out float poppedF);
            if (closed.Contains(ck)) continue;
            if (!recs.TryGetValue(ck, out var cr)) continue;
            if (poppedF > cr.f + 1e-4f) continue;

            closed.Add(ck);

            if (Dist(cr.p, g) <= goalTol || SameCell(cr.p, g))
                return Reconstruct(ck);

            for (int si = 0; si < S; si++)
            {
                float t = (float)si / (S - 1);
                float steer = Mathf.Lerp(-maxSteer, maxSteer, t);

                Expand(ck, cr, steer, false, g, open);
                if (allowReverse) Expand(ck, cr, steer, true, g, open);
            }
        }
        return null;
    }

    void Expand(int ck, Rec cr, float steer, bool rev, Vector3 goal, Heap open)
    {
        if (!Propagate(cr.p, cr.theta, steer, rev, out var np, out var nt)) return;
        int nk = Key(np, nt);
        if (closed.Contains(nk)) return;

        float stepCost = stepSize + 0.03f * Mathf.Abs(steer) + 0.05f * Mathf.Abs(steer - cr.lastSteer) + (rev ? 3.0f : 0f);
        float ng = cr.g + stepCost;

        if (recs.TryGetValue(nk, out var old) && ng >= old.g) return;

        float h = Dist(np, goal);
        recs[nk] = new Rec { g = ng, f = ng + h, parent = ck, p = np, theta = nt, lastSteer = steer };
        open.Push(nk, ng + h);
    }

    bool Propagate(Vector3 p, float th, float steer, bool rev, out Vector3 ep, out float eth)
    {
        ep = p; eth = th;
        float dir = rev ? -1f : 1f;

        float subTarget = Mathf.Max(0.05f, cellSize * 0.5f);
        int steps = Mathf.Max(3, Mathf.CeilToInt(stepSize / subTarget));
        float ds = stepSize / steps;

        if (!InBounds(ep)) return false;

        for (int i = 0; i < steps; i++)
        {
            float omega = (dir * ds) * Mathf.Tan(steer) / Mathf.Max(0.01f, wheelBase);
            float mid = eth + 0.5f * omega;

            ep.x += dir * ds * Mathf.Sin(mid);
            ep.z += dir * ds * Mathf.Cos(mid);
            eth = Wrap(eth + omega);

            if (!InBounds(ep)) return false;
            if (i == steps / 2 || i == steps - 1)
                if (!PoseFree(ep, eth)) return false;
        }
        return true;
    }

    // follower helpers
    void ClosestOnPath(out int seg, out Vector3 cp, out float signedCTE)
    {
        seg = 0; cp = pathW[0]; signedCTE = 0f;

        var steerPoint = transform.position + transform.forward * (wheelBase * 0.5f);
        Vector2 P = new(steerPoint.x, steerPoint.z);

        float best = float.PositiveInfinity;
        int i0 = Mathf.Clamp(pathIdx, 0, pathW.Count - 2);
        int i1 = Mathf.Clamp(pathIdx + 30, 0, pathW.Count - 2);

        for (int i = i0; i <= i1; i++)
        {
            Vector2 A = new(pathW[i].x, pathW[i].z);
            Vector2 B = new(pathW[i + 1].x, pathW[i + 1].z);
            Vector2 AB = B - A;
            float ab2 = Vector2.Dot(AB, AB);
            if (ab2 < 1e-6f) continue;

            float t = Mathf.Clamp01(Vector2.Dot(P - A, AB) / ab2);
            Vector2 C = A + t * AB;

            Vector2 E = P - C;
            float d2 = E.sqrMagnitude;
            if (d2 >= best) continue;

            best = d2;
            seg = i;
            cp = new Vector3(C.x, transform.position.y, C.y);

            float cross = AB.x * E.y - AB.y * E.x;
            signedCTE = (cross > 0f ? -1f : 1f) * Mathf.Sqrt(d2);
        }

        seg = Mathf.Clamp(seg, 0, pathW.Count - 2);
    }

    Vector3 LookaheadPoint(int segIdx, Vector3 from, float look)
    {
        int i = Mathf.Clamp(segIdx, 0, pathW.Count - 2);
        Vector3 cur = from; cur.y = 0f;
        float rem = Mathf.Max(0.01f, look);

        while (i < pathW.Count - 1)
        {
            Vector3 a = (i == segIdx) ? cur : pathW[i];
            Vector3 b = pathW[i + 1];
            a.y = 0f; b.y = 0f;

            Vector3 ab = b - a;
            float len = ab.magnitude;
            if (len < 1e-4f) { i++; continue; }

            if (rem <= len) return a + ab * (rem / len);
            rem -= len;
            i++;
        }
        var last = pathW[^1]; last.y = 0f; return last;
    }

    float UpcomingTurnDeg(int segIdx, Vector3 closest, out float distToMax)
    {
        distToMax = 0f;
        if (pathW.Count < 3) return 0f;

        int i0 = Mathf.Clamp(segIdx, 0, pathW.Count - 2);
        Vector3 A0 = closest; A0.y = 0f;
        Vector3 B0 = pathW[i0 + 1]; B0.y = 0f;
        Vector3 d0 = (B0 - A0);
        if (d0.sqrMagnitude < 1e-6f) return 0f;
        d0.Normalize();

        float traveled = Vector3.Distance(A0, B0);
        float bestAng = 0f, bestDist = traveled;

        for (int i = i0 + 1; i < pathW.Count - 1; i++)
        {
            Vector3 a = pathW[i]; a.y = 0f;
            Vector3 b = pathW[i + 1]; b.y = 0f;

            float segLen = Vector3.Distance(a, b);
            if (segLen < 1e-4f) continue;

            traveled += segLen;
            if (traveled > Mathf.Max(0.1f, turnLookahead)) break;

            Vector3 di = (b - a);
            if (di.sqrMagnitude < 1e-6f) continue;
            di.Normalize();

            float ang = Mathf.Abs(Vector3.SignedAngle(d0, di, Vector3.up));
            if (ang > bestAng) { bestAng = ang; bestDist = traveled; }
        }

        distToMax = bestDist;
        return bestAng;
    }

    // collision check 
    bool PoseFree(Vector3 pLocal, float theta)
    {
        float r = Mathf.Max(0.05f, inflateR);
        float pr = Mathf.Max(0.05f, r * Mathf.Clamp(probeScale, 0.3f, 1.2f));

        Vector3 cW = MapManager.transform.TransformPoint(pLocal);
        cW.y += checkY;

        if (Physics.CheckSphere(cW, r, obstacleMask, QueryTriggerInteraction.Ignore)) return false;

        float s = Mathf.Sin(theta), c = Mathf.Cos(theta);
        Vector3 fwdL = new(s, 0f, c);
        Vector3 rightL = new(c, 0f, -s);
        Vector3 fwdW = MapManager.transform.TransformDirection(fwdL);
        Vector3 rightW = MapManager.transform.TransformDirection(rightL);

        if (Physics.CheckSphere(cW + fwdW * probeFwd, pr, obstacleMask)) return false;
        if (Physics.CheckSphere(cW - fwdW * probeFwd, pr, obstacleMask)) return false;
        if (Physics.CheckSphere(cW + rightW * probeSide, pr, obstacleMask)) return false;
        if (Physics.CheckSphere(cW - rightW * probeSide, pr, obstacleMask)) return false;

        return true;
    }

    bool InBounds(Vector3 p)
    {
        return !(p.x < bounds.min.x || p.x > bounds.max.x || p.z < bounds.min.z || p.z > bounds.max.z);
    }

    // discretization / misc
    int Key(Vector3 p, float theta)
    {
        float cs = Mathf.Max(0.01f, cellSize);
        int ix = Mathf.Clamp(Mathf.FloorToInt((p.x - originLocal.x) / cs), 0, W - 1);
        int iz = Mathf.Clamp(Mathf.FloorToInt((p.z - originLocal.z) / cs), 0, H - 1);
        int it = Mathf.Clamp(Mathf.FloorToInt((Wrap(theta) / (2f * Mathf.PI)) * thetaBins), 0, thetaBins - 1);
        return ix + W * (iz + H * it);
    }

    bool SameCell(Vector3 p, Vector3 g)
    {
        float cs = Mathf.Max(0.01f, cellSize);
        int ix = Mathf.FloorToInt((p.x - originLocal.x) / cs);
        int iz = Mathf.FloorToInt((p.z - originLocal.z) / cs);
        int gx = Mathf.FloorToInt((g.x - originLocal.x) / cs);
        int gz = Mathf.FloorToInt((g.z - originLocal.z) / cs);
        return ix == gx && iz == gz;
    }

    static float Dist(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x, dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    static float Wrap(float a)
    {
        float twoPi = 2f * Mathf.PI;
        a %= twoPi;
        if (a < 0f) a += twoPi;
        return a;
    }

    List<Rec> Reconstruct(int goalKey)
    {
        var path = new List<Rec>(1024);
        int cur = goalKey;
        for (int guard = 0; cur != -1 && guard < 1_000_000; guard++)
        {
            if (!recs.TryGetValue(cur, out var r)) break;
            path.Add(r);
            cur = r.parent;
        }
        path.Reverse();
        return path;
    }
}
