using System;
using System.Collections.Generic;
using System.Transactions;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

[RequireComponent(typeof(CarController))]
public class CarAI : Agent
{
    // An example class, containing code snippets demonstrating how to do different things in the environment.
    private Vector3 _debugLookaheadTarget;
    private Vector3 _debugCurrentWp;
    public CarController car; // the car controller we want to use
    private BoxCollider m_CarCollider;

    // Parameters
    private const float CellSize = 1.0f;
    private const float InflationRadius = 2.0f;
    private const float WaypointReachDist = 2.5f; // Distance to check if goal has been reached
    private const float LookaheadDist = 1.0f;
    private List<Vector3> path = new List<Vector3>(); // Final path from A*
    private int pathid = 0; // number of node in path 
    private BoundsInt _localBounds;
    private Vector3[] _inflationOffsets;

    private struct Cell : IEquatable<Cell> // Definition of a cell
    {
        public int ix, iz;
        public Cell(int ix, int iz) { this.ix = ix; this.iz = iz; }
        public bool Equals(Cell other) => ix == other.ix && iz == other.iz;
        public override bool Equals(object obj) => obj is Cell other && Equals(other);
        public override int GetHashCode() => (ix * 73856093) ^ (iz * 19349663);
    }

    public override void Initialize()  // Runs before game starts
    {
        //How to fetch components from the game hierarchy
        m_CarCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();

        /////// Plan your path here  ////////
        Vector3 someLocalPosition = MapManager.transform.InverseTransformPoint(transform.position); // Position of car w.r.p map coordinate origin (not world global)

        _localBounds = ObstacleMap.localBounds;

        _inflationOffsets = new[]
        {
            Vector3.zero,
            new Vector3( InflationRadius, 0, 0),
            new Vector3(-InflationRadius, 0, 0),
            new Vector3(0, 0,  InflationRadius),
            new Vector3(0, 0, -InflationRadius),
            new Vector3( InflationRadius, 0,  InflationRadius),
            new Vector3( InflationRadius, 0, -InflationRadius),
            new Vector3(-InflationRadius, 0,  InflationRadius),
            new Vector3(-InflationRadius, 0, -InflationRadius),
        };

        // Replace the code below that makes a random path
        // ...

        Vector3 startpos = MapManager.GetGlobalStartPosition(); // Start & Goal positions in map coordinates
        Vector3 goalpos = MapManager.GetGlobalGoalPosition();
        
        Vector3 startLocal = MapManager.transform.InverseTransformPoint(startpos); // Start & Goal positions in map local coordinates
        Vector3 goalLocal = MapManager.transform.InverseTransformPoint(goalpos);
        // Run A*
        List<Vector3> rawPathWorld = PlanAStarWorld(startLocal, goalLocal);

        if (rawPathWorld == null || rawPathWorld.Count == 0)
        {
            Debug.LogWarning("A* returned no path. Check inflation/cell size or map bounds.");
            path = new List<Vector3> { startpos, goalpos };
        }
        else
        {
            // Optional but strongly recommended: simplify path (removes grid zig-zag)
            path = TransformPath(rawPathWorld);
        }

        pathid = 0;

        // Draw final path for a long time (Scene view)
        DrawPath(path, Color.white, 99999f);
    }


    public override void Step() // Runs every step of the physics simulation
    {
        // Execute your path here
        // ...
        if (path == null || path.Count == 0)
        {
            car.Move(0f, 0f, 1f, 0f); // car.Move(steer, accel, footbrake, handbrake)
        }
        Vector3 position = transform.position;

        if (pathid >= path.Count - 1)
        {
            float dGoal = Vector3.Distance(position, path[path.Count - 1]);
            if (dGoal < WaypointReachDist)
            {
                car.Move(0f, 0f, 1f, 0f); // brake if close to goal
                return;
            }
        }

        while (pathid < path.Count - 1 &&
               Vector3.Distance(position, path[pathid]) < WaypointReachDist)
        {
            pathid++; // Advance waypoint if close
        }

        Vector3 target = GetLookaheadTarget(position, path, pathid, LookaheadDist);  // Create target to track
        Vector3 toTarget = target - position;
        toTarget.y = 0f;
        if (toTarget.sqrMagnitude < 0.001f)
        {
            car.Move(0f, 0f, 1f, 0f); // if the car is close to the target, return
            return;
        }
        Vector3 forward = transform.forward; forward.y = 0f;
        forward.Normalize();
        toTarget.Normalize();

        float steeringAngle = Vector3.SignedAngle(forward, toTarget, Vector3.up);

        // Convert angle to steer [-1,1] (tune "max steer angle" mapping)
        float maxAngleDeg = 45f;
        float steer = Mathf.Clamp(steeringAngle / maxAngleDeg, -1f, 1f);

        // Simple speed policy: slow down when turning sharply
        float accel = Mathf.Lerp(0.5f, 0.3f, Mathf.Abs(steer));
        float footbrake = 0f;

        car.Move(steer, accel, footbrake, 0f);

        Debug.DrawLine(position, target, Color.cyan, 0f);
        // this is how you control the car
    }
    // Function to draw the path returned by A*
    private void DrawPath(List<Vector3> path, Color c, float duration) 
    {
        if (path == null || path.Count < 2) return;
        for (int i = 0; i < path.Count - 1; i++)
        {
            Debug.DrawLine(path[i], path[i + 1], c, duration);
        }
    }
    // Function to transform the path to a drivable path (To be implemented)
    private List<Vector3> TransformPath(List<Vector3> pathWorld)
    {
        if (pathWorld == null || pathWorld.Count < 3) return pathWorld;

        var simplified = new List<Vector3>();
        int i = 0;
        simplified.Add(pathWorld[0]);

        while (i < pathWorld.Count - 1)
        {
            int best = i + 1;

            // try to jump as far as possible while keeping line-of-sight free
            for (int j = pathWorld.Count - 1; j > best; j--)
            {
                if (SegmentIsFree(pathWorld[i], pathWorld[j]))
                {
                    best = j;
                    break;
                }
            }

            simplified.Add(pathWorld[best]);
            i = best;
        }

        return simplified;
    }

        private bool SegmentIsFree(Vector3 aWorld, Vector3 bWorld)
    {
        // Sample along the segment in map-local space
        Vector3 aLocal = MapManager.transform.InverseTransformPoint(aWorld);
        Vector3 bLocal = MapManager.transform.InverseTransformPoint(bWorld);

        Vector3 d = bLocal - aLocal;
        d.y = 0f;
        float len = d.magnitude;
        if (len < 0.001f) return true;

        Vector3 dir = d / len;

        float step = CellSize * 0.5f; // conservative sampling
        int n = Mathf.CeilToInt(len / step);

        for (int k = 0; k <= n; k++)
        {
            float s = (n == 0) ? 0f : (k / (float)n);
            Vector3 p = Vector3.Lerp(aLocal, bLocal, s);

            foreach (var off in _inflationOffsets)
            {
                var t = ObstacleMap.GetLocalPointTraversibility(p + off);
                if (t == ObstacleMap.Traversability.Blocked)
                    return false;
            }
        }

        return true;
    }

    // Get the target ahead
    private Vector3 GetLookaheadTarget(Vector3 pos, List<Vector3> path, int startIdx, float lookahead)
{
    if (path == null || path.Count == 0) return pos;
    if (path.Count == 1) return path[0];

    int i0 = Mathf.Clamp(startIdx, 0, path.Count - 1);

    float remaining = lookahead;

    for (int i = i0; i < path.Count - 1; i++)
    {
        Vector3 a = path[i];
        Vector3 b = path[i + 1];

        Vector3 ab = b - a;
        float segLen = ab.magnitude;
        if (segLen < 1e-4f) continue;

        if (segLen >= remaining)
            return a + ab.normalized * remaining;

        remaining -= segLen;
    }

    return path[path.Count - 1];
}

    // A* Planner
    private List<Vector3> PlanAStarWorld(Vector3 startLocal, Vector3 goalLocal)
    {
        Cell start = LocalToCell(startLocal);
        Cell goal = LocalToCell(goalLocal);
        if (IsCellBlocked(start) || IsCellBlocked(goal))
        {
            Debug.LogWarning("Start or goal cell is occupied.");
            return null;
        }
        var frontier = new List<Cell>();  // List of frontier cells to explore
        var frontier_check = new HashSet<Cell>(); // Uses hashes to check frontier set
        var cameFrom = new Dictionary<Cell, Cell>(); // Pointer to parent cell
        var cost = new Dictionary<Cell, float>(); // Cost to reach the cell
        frontier.Add(start);
        frontier_check.Add(start);
        cost[start] = 0f;
        // 8-neighbors (dx, dz, cost)
        (int dx, int dz, float cost)[] moves =
        {
            ( 1, 0, 1f), (-1, 0, 1f), (0, 1, 1f), (0,-1, 1f),
            ( 1, 1, 1.4142f), ( 1,-1, 1.4142f), (-1, 1, 1.4142f), (-1,-1, 1.4142f)
        };
        int safetyIter = 0;
        int safetyMax = 200000; // prevents infinite loops
        while(frontier.Count > 0  && safetyIter++ < safetyMax)
        {
            // We pick the node in frontier set with the smallest cost
            int bestIdx = 0;
            float bestF = float.PositiveInfinity;
            
            for (int i = 0; i < frontier.Count; i++)
            {
                Cell c = frontier[i];
                float cost_node;
                if(cost.TryGetValue(c, out float gv))
                {
                    cost_node = gv;
                }
                else
                {
                    cost_node = float.PositiveInfinity;
                }
                float heuristic = Heuristic(c, goal);
                float cost_total = cost_node + heuristic;

                if (cost_total < bestF)
                {
                    bestF = cost_total;
                    bestIdx = i;
                }
            }
            Cell current = frontier[bestIdx];
            frontier.RemoveAt(bestIdx);
            frontier_check.Remove(current);
            if (current.Equals(goal))
            {
                // Reconstruct
                List<Cell> cellPath = Reconstruct(cameFrom, current);
                return CellsToWorldWaypoints(cellPath);
            }
            foreach (var m in moves)
            {
                Cell neighbor_cell = new Cell(current.ix + m.dx, current.iz + m.dz);

                if (!IsCellInBounds(neighbor_cell)) continue;
                if (IsCellBlocked(neighbor_cell)) continue;

                float currentG = cost.TryGetValue(current, out var cg) ? cg : float.PositiveInfinity;
                float tentativeG = currentG + (m.cost * CellSize);

                float neighbor_cost = cost.TryGetValue(neighbor_cell, out var ng) ? ng : float.PositiveInfinity;
                if (tentativeG < neighbor_cost)
                {
                    cameFrom[neighbor_cell] = current;
                    cost[neighbor_cell] = tentativeG;

                    if (!frontier_check.Contains(neighbor_cell))
                    {
                        frontier.Add(neighbor_cell);
                        frontier_check.Add(neighbor_cell);
                    }
                }
            }
        }     
        
        return null;
    }
    private Cell LocalToCell(Vector3 local)
    {
        float minX = _localBounds.min.x;
        float minZ = _localBounds.min.z;

        int ix = Mathf.FloorToInt((local.x - minX) / CellSize);
        int iz = Mathf.FloorToInt((local.z - minZ) / CellSize);

        return new Cell(ix, iz);
    }
    private Vector3 CellToLocalCenter(Cell c)
    {
        float minX = _localBounds.min.x;
        float minZ = _localBounds.min.z;

        float x = minX + (c.ix + 0.5f) * CellSize;
        float z = minZ + (c.iz + 0.5f) * CellSize;

        return new Vector3(x, 0f, z);
    }
    // Occupied cell checker
    private bool IsCellBlocked(Cell c)
    {
        Vector3 centerLocal = CellToLocalCenter(c);

        // Inflation: if any sample point is blocked, cell is blocked
        foreach (var off in _inflationOffsets)
        {
            var t = ObstacleMap.GetLocalPointTraversibility(centerLocal + off);
            if (t == ObstacleMap.Traversability.Blocked)
                return true;
        }

        return false;
    }
    private float Heuristic(Cell a, Cell b)
    {
        // Euclidean distance in grid scaled by CellSize
        float dx = (a.ix - b.ix) * CellSize;
        float dz = (a.iz - b.iz) * CellSize;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }
    private List<Cell> Reconstruct(Dictionary<Cell, Cell> cameFrom, Cell current)
    {
        var path = new List<Cell> { current };
        while (cameFrom.TryGetValue(current, out var parent))
        {
            current = parent;
            path.Add(current);
        }
        path.Reverse();
        return path;
    }
    private List<Vector3> CellsToWorldWaypoints(List<Cell> cellPath)
    {
        var outWorld = new List<Vector3>(cellPath.Count);
        for (int i = 0; i < cellPath.Count; i++)
        {
            Vector3 local = CellToLocalCenter(cellPath[i]);
            Vector3 world = MapManager.transform.TransformPoint(local);
            outWorld.Add(world);
        }
        return outWorld;
    }
    private bool IsCellInBounds(Cell c)
    {
        Vector3 p = CellToLocalCenter(c);

        float minX = _localBounds.min.x;
        float minZ = _localBounds.min.z;
        float maxX = _localBounds.min.x + _localBounds.size.x;
        float maxZ = _localBounds.min.z + _localBounds.size.z;

        return (p.x >= minX && p.x <= maxX &&
                p.z >= minZ && p.z <= maxZ);
    }
}