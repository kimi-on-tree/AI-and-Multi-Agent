using System.Collections.Generic;
using Scripts.Game;
using Scripts.Vehicle;
using Scripts.Map;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : Agent
{
    public DroneController m_Drone;

    // How big each cell is in meters
    [Tooltip("Cellsize, basically resolution.")]
    public float cellSize = 1f;

    [Tooltip("Drone hald width, used as clearance for objects.")]
    public float droneHalfWidthMeters = 1f;

    // Start/goal in world space
    private Vector3 startWorld, goalWorld;

    // Start/goal converted to ObstacleMap cell coordinates 
    private Vector3Int startCell, goalCell;

    // Path stored in cell cordinates
    private List<Vector3Int> pathCells = new List<Vector3Int>();

    // For path finding
    private List<Vector3> pathWorld = new List<Vector3>();
    private int pathIndex = 0;

    [Tooltip("Distance to node for it to be reached")]
    public float waypointRadius = 1f;

    // Extra safety margin for overhang objects
    private const float extraObjectMarginMeters = 1.5f;
    private static readonly string[] ObjectNamePrefixes =
    {
        "tree_pineSmallC","tree_detailed_dark","tree_pineRoundF",
        "tree_plateau","tree_oak_dark","house_type02","large_buildingA"
    };

    private bool extraMarginWalls0 = true;
    private float extraWallMarginMeters = 1.5f;
    private string wallsRootName = "Walls 0";
    private readonly List<Bounds> m_TreeBounds = new List<Bounds>(512);
    private readonly List<Bounds> m_WallBounds = new List<Bounds>(256);

    public override void Initialize()
    {
        // Da drone
        m_Drone = GetComponent<DroneController>();

        // Create the ObstacleMap
        ObstacleMap = ObstacleMap.Initialize(
            MapManager,
            new System.Collections.Generic.List<GameObject>(),
            new Vector3(cellSize, 1f, cellSize)
        );

        CacheOverhangObjectBounds();
        CacheWallBounds();

        // Fetch start/goal positions
        startWorld = MapManager.GetGlobalStartPosition();
        goalWorld = MapManager.GetGlobalGoalPosition();
        // Convert world positions into CELL coordinates used for path finding later
        startCell = ObstacleMap.WorldToCell(startWorld);
        goalCell = ObstacleMap.WorldToCell(goalWorld);

        // Debug stuff
        Debug.Log($"StartWorld: {startWorld}, GoalWorld: {goalWorld}");
        Debug.Log($"StartCell: {startCell}, GoalCell: {goalCell}");
        Debug.Log($"Traversability cells: {ObstacleMap.traversabilityPerCell.Count}");

        // Plan path with A*
        pathCells = AStar(startCell, goalCell);
        Debug.Log($"A* path length (cells): {pathCells.Count}");

        // Prune the basic A* path
        pathCells = SimplifyAStarPath(pathCells);
        Debug.Log($"Simplified path length (cells): {pathCells.Count}");

        pathWorld.Clear();
        foreach (var c in pathCells)
            pathWorld.Add(ObstacleMap.CellToWorld(c) + ObstacleMap.trueScale / 2f);

        pathIndex = 0;
        Debug.Log($"PathWorld count: {pathWorld.Count}");
        Debug.Log($"First target: {(pathWorld.Count > 0 ? pathWorld[0].ToString() : "NONE")}");
    }


    // Check if a cell is valid + not blocked
    private bool IsFree(Vector3Int c)
    {
        if (!ObstacleMap.cellBounds.Contains(c)) return false;

        // If the center is not traversable then skip fast so we dont calculate too long
        var centerKey = new Vector2Int(c.x, c.z);
        if (!ObstacleMap.traversabilityPerCell.TryGetValue(centerKey, out var centerT)) return false;
        if (centerT == ObstacleMap.Traversability.Blocked) return false;

        float radius = droneHalfWidthMeters;
        float step = cellSize * 0.5f;

        // Center of this cell in world
        Vector3 centerWorld = ObstacleMap.CellToWorld(c) + ObstacleMap.trueScale / 2f;

        // extra inflation, on top of drone width
        if (m_TreeBounds.Count > 0 &&
            TooCloseToBounds(m_TreeBounds, centerWorld, droneHalfWidthMeters + extraObjectMarginMeters))
            return false;

        if (extraMarginWalls0 && m_WallBounds.Count > 0 &&
            TooCloseToBounds(m_WallBounds, centerWorld, droneHalfWidthMeters + extraWallMarginMeters))
            return false;

        for (float dx = -radius; dx <= radius; dx += step)
        {
            for (float dz = -radius; dz <= radius; dz += step)
            {
                // Square footprint sampling
                Vector3 p = centerWorld + new Vector3(dx, 0f, dz);

                Vector3Int cc = ObstacleMap.WorldToCell(p);
                if (!ObstacleMap.cellBounds.Contains(cc)) return false;

                var key = new Vector2Int(cc.x, cc.z);
                if (!ObstacleMap.traversabilityPerCell.TryGetValue(key, out var t)) return false;
                if (t == ObstacleMap.Traversability.Blocked) return false;
            }
        }

        return true;
    }

    // A positions 4-connected neighbors (up/down/left/right)
    private List<Vector3Int> Neighbors4(Vector3Int c) => new List<Vector3Int>
    {
        new Vector3Int(c.x + 1, 0, c.z),
        new Vector3Int(c.x - 1, 0, c.z),
        new Vector3Int(c.x, 0, c.z + 1),
        new Vector3Int(c.x, 0, c.z - 1)
    };

    // Heuristic for A* (used as distance to goal)
    private float Heuristic(Vector3Int a, Vector3Int b)
    {
        float dx = a.x - b.x;
        float dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    // Rebuild the path by going backwards
    private List<Vector3Int> ReconstructPath(Dictionary<Vector3Int, Vector3Int> cameFrom, Vector3Int current)
    {
        var path = new List<Vector3Int> { current };
        while (cameFrom.TryGetValue(current, out var prev))
        {
            current = prev;
            path.Add(current);
        }
        path.Reverse();
        return path;
    }

    // A* path planner
    private List<Vector3Int> AStar(Vector3Int start, Vector3Int goal)
    {
        var open = new List<Vector3Int> { start };
        var cameFrom = new Dictionary<Vector3Int, Vector3Int>();
        var gScore = new Dictionary<Vector3Int, float> { [start] = 0f };

        while (open.Count > 0)
        {
            int bestIdx = 0;
            float bestF = float.PositiveInfinity;

            for (int i = 0; i < open.Count; i++)
            {
                var n = open[i];
                float g = gScore.TryGetValue(n, out float gv) ? gv : float.PositiveInfinity;
                float f = g + Heuristic(n, goal);
                if (f < bestF) { bestF = f; bestIdx = i; }
            }

            var current = open[bestIdx];
            open.RemoveAt(bestIdx);

            // Goal check
            if (current.x == goal.x && current.z == goal.z)
                return ReconstructPath(cameFrom, current);

            foreach (var nb in Neighbors4(current))
            {
                if (!IsFree(nb)) continue;

                float tentativeG = gScore[current] + 1f;
                if (!gScore.TryGetValue(nb, out float oldG) || tentativeG < oldG)
                {
                    cameFrom[nb] = current;
                    gScore[nb] = tentativeG;

                    if (!open.Contains(nb)) open.Add(nb);
                }
            }
        }
        return new List<Vector3Int>();
    }


    private List<Vector3Int> SimplifyAStarPath(List<Vector3Int> inputPath)
    {
        var simplified = new List<Vector3Int>();
        if (inputPath == null || inputPath.Count == 0) return simplified;

        int n = inputPath.Count, anchor = 0;
        simplified.Add(inputPath[anchor]);

        while (anchor < n - 1)
        {
            // Find furthest point visible from anchor
            int lastGood = anchor + 1;
            for (int test = anchor + 1; test < n; test++)
                if (HasClearPath(inputPath[anchor], inputPath[test])) lastGood = test;

            // SKip it if we cant see further than next point
            if (lastGood == anchor + 1) { simplified.Add(inputPath[lastGood]); anchor = lastGood; continue; }

            // Try all pivots between the anchor and lastGood to see if they can jump farther than lastGood. Gave big improvement on many maps
            int bestPivot = -1, bestReach = lastGood;

            for (int pivot = anchor + 1; pivot < lastGood; pivot++)
            {
                int pivotReach = pivot + 1;

                // Find furthest point visible from this pivot
                for (int test = pivot + 1; test < n; test++)
                    if (HasClearPath(inputPath[pivot], inputPath[test])) pivotReach = test;

                // If pivot reaches farther than the anchor did then we set that as the best node in our path making
                if (pivotReach > bestReach) { bestReach = pivotReach; bestPivot = pivot; }
            }

            // Commit the result we found(if there is one)
            if (bestPivot != -1)
            {
                if (simplified[simplified.Count - 1] != inputPath[bestPivot]) simplified.Add(inputPath[bestPivot]);
                if (simplified[simplified.Count - 1] != inputPath[bestReach]) simplified.Add(inputPath[bestReach]);
                anchor = bestReach;
            }
            else { simplified.Add(inputPath[lastGood]); anchor = lastGood; }
        }

        return simplified;
    }

    // Check if a path between 2 cells is free with IsFree()
    private bool HasClearPath(Vector3Int aCell, Vector3Int bCell)
    {
        Vector3 a = ObstacleMap.CellToWorld(aCell) + ObstacleMap.trueScale / 2f;
        Vector3 b = ObstacleMap.CellToWorld(bCell) + ObstacleMap.trueScale / 2f;

        Vector3 ab = b - a;
        ab.y = 0f;

        float len = ab.magnitude;
        if (len < 0.001f) return true;

        Vector3 dir = ab / len;

        float step = cellSize * 0.5f; 

        for (float d = 0f; d <= len; d += step)
        {
            Vector3 p = a + dir * d;

            if (m_TreeBounds.Count > 0 &&
                TooCloseToBounds(m_TreeBounds, p, droneHalfWidthMeters + extraObjectMarginMeters))
                return false;

            if (extraMarginWalls0 && m_WallBounds.Count > 0 &&
                TooCloseToBounds(m_WallBounds, p, droneHalfWidthMeters + extraWallMarginMeters))
                return false;

            if (!IsFree(ObstacleMap.WorldToCell(p))) return false;
        }

        return true;
    }


    private void CacheOverhangObjectBounds()
    {
        m_TreeBounds.Clear();

        foreach (Transform t in MapManager.GetComponentsInChildren<Transform>(true))
        {
            string n = t.name;
            bool match = false;
            for (int i = 0; i < ObjectNamePrefixes.Length; i++)
                if (n.StartsWith(ObjectNamePrefixes[i])) { match = true; break; }
            if (!match) continue;

            var cols = t.GetComponentsInChildren<Collider>(true);
            for (int i = 0; i < cols.Length; i++)
                if (cols[i] != null) m_TreeBounds.Add(cols[i].bounds);
        }

        Debug.Log($"[DroneAI] Cached {m_TreeBounds.Count} tree collider bounds.");
    }

    private void CacheWallBounds()
    {
        m_WallBounds.Clear();
        if (!extraMarginWalls0) return;

        Transform wallsRoot = MapManager.transform.Find(wallsRootName);
        if (wallsRoot == null)
        {
            Transform mapT = MapManager.transform.Find("Map");
            if (mapT != null) wallsRoot = mapT.Find(wallsRootName);
        }
        if (wallsRoot == null)
        {
            foreach (Transform t in MapManager.GetComponentsInChildren<Transform>(true))
                if (t.name == wallsRootName) { wallsRoot = t; break; }
        }

        if (wallsRoot == null)
        {
            Debug.LogWarning($"[DroneAI] Could not find '{wallsRootName}' under MapManager. No extra wall margin applied.");
            return;
        }

        var cols = wallsRoot.GetComponentsInChildren<Collider>(true);
        for (int i = 0; i < cols.Length; i++)
            if (cols[i] != null) m_WallBounds.Add(cols[i].bounds);

        Debug.Log($"[DroneAI] Cached {m_WallBounds.Count} wall collider bounds under '{wallsRootName}'.");
    }

    // extra margin check for risky objects
    private static bool TooCloseToBounds(List<Bounds> boundsList, Vector3 p, float requiredClearanceMeters)
    {
        float rr = requiredClearanceMeters * requiredClearanceMeters;

        for (int i = 0; i < boundsList.Count; i++)
        {
            Bounds b = boundsList[i];

            float dx = (p.x < b.min.x) ? (b.min.x - p.x) : (p.x > b.max.x ? p.x - b.max.x : 0f);
            float dz = (p.z < b.min.z) ? (b.min.z - p.z) : (p.z > b.max.z ? p.z - b.max.z : 0f);

            if (dx * dx + dz * dz < rr) return true;
        }
        return false;
    }

    // Mark obstacles with red
    void OnDrawGizmos()
    {
        if (ObstacleMap == null || ObstacleMap.traversabilityPerCell == null) return;

        // Draw blocked cells as red squares
        foreach (var kv in ObstacleMap.traversabilityPerCell)
        {
            if (kv.Value != ObstacleMap.Traversability.Blocked) continue; // Only draw on the blocked ones, maybe we draw green on open spots?

            Vector3 cellCenterWorld =
                ObstacleMap.CellToWorld(new Vector3Int(kv.Key.x, 0, kv.Key.y)) + ObstacleMap.trueScale / 2f;

            Gizmos.color = new Color(1f, 0f, 0f, 0.35f);
            Gizmos.DrawCube(
                cellCenterWorld,
                new Vector3(ObstacleMap.trueScale.x, 0.05f, ObstacleMap.trueScale.z)
            );
        }

        // Draw the A* path as white lines
        if (pathCells != null && pathCells.Count > 1)
        {
            Gizmos.color = Color.white;
            for (int i = 1; i < pathCells.Count; i++)
            {
                Vector3 a = ObstacleMap.CellToWorld(pathCells[i - 1]) + ObstacleMap.trueScale / 2f;
                Vector3 b = ObstacleMap.CellToWorld(pathCells[i]) + ObstacleMap.trueScale / 2f;
                Gizmos.DrawLine(a, b);
            }
        }
    }

    public override void Step()
    {
        if (pathWorld == null || pathWorld.Count == 0 || pathIndex >= pathWorld.Count)
        {
            m_Drone.Move(0f, 0f);
            return;
        }

        Rigidbody rb = GetComponent<Rigidbody>();
        Vector3 vel = rb.linearVelocity;
        vel.y = 0f;

        Vector3 pos = transform.position;
        Vector3 target = pathWorld[pathIndex];

        Debug.DrawLine(pos, target, Color.cyan);

        Vector3 toTarget = target - pos;
        toTarget.y = 0f;

        float dist = toTarget.magnitude;
        if (dist < 0.001f)
        {
            m_Drone.Move(0f, 0f);
            return;
        }

        Vector3 dir = toTarget / dist;

        // speed component TOWARD the target
        float vToward = Vector3.Dot(vel, dir);

        // Reached waypoint check
        if (dist < waypointRadius)
        {
            pathIndex++;
            m_Drone.Move(0f, 0f);
            return;
        }

        float aMax = m_Drone.max_acceleration;
        float vMax = m_Drone.max_speed;

        // Split velocity into along and side relative to the waypoint direction
        Vector3 velAlong = dir * vToward;
        Vector3 velSide = vel - velAlong;        

        // SIDEWAYS damping, to keep being on the line
        float tauSide = 0.35f;                   
        Vector3 aSide = -velSide / tauSide;

        // Clamp sideways accel
        if (aSide.magnitude > aMax)
            aSide = aSide.normalized * aMax;

        // ALONG-direction
        float vTowardPos = vToward;
        if (vTowardPos < 0f) vTowardPos = 0f;

        float stopBuffer = waypointRadius * 0.75f;
        float distToStop = Mathf.Max(0f, dist - stopBuffer);

        float stopDistNeeded = (vTowardPos * vTowardPos) / (2f * aMax);

        Vector3 aAlong;

        if (stopDistNeeded >= distToStop && vel.sqrMagnitude > 0.0001f)
        {
            // brake hard only in the path direction
            aAlong = -dir * aMax;
        }
        else
        {
            // accelerate hard toward waypoint but not going past VMax
            if (vTowardPos >= vMax * 0.98f) aAlong = Vector3.zero;
            else aAlong = dir * aMax;
        }

        // Combine and clamp
        Vector3 aCmd = aAlong + aSide;

        if (aCmd.magnitude > aMax)
            aCmd = aCmd.normalized * aMax;

        float hCmd = Mathf.Clamp(aCmd.x / aMax, -1f, 1f);
        float vCmd = Mathf.Clamp(aCmd.z / aMax, -1f, 1f);

        m_Drone.Move(hCmd, vCmd);
    }
}
