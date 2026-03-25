using System.Collections.Generic;
using System.Linq;
using Scripts.Map;
using UnityEngine;

namespace Path_Planning
{
    public class AStarPlanner : PathPlanner
    {

        private Dictionary<Vector3Int, Vector2> _grid;
        private ObstacleMap _obstacleMap;
        private ObstacleRiskGrid _obstacleRiskGrid;
        private static readonly int GridResolution = 4;
        private static readonly int ObstacleInflation = 1;

        public AStarPlanner(MapManager mapManager, Vector3 startPos, Vector3 goalPos, VehicleDynamics vehicleDynamics) :
            base(mapManager, startPos, goalPos, vehicleDynamics)
        {
            _obstacleMap =  ObstacleMap.Initialize(MapManager,
                additionalObjects: new List<GameObject>(),
                cellScale: Vector3.one * GridResolution,
                margin: Vector3.one);
        }

        public AStarPlanner(MapManager mapManager, Vector3 startPos, Vector3 goalPos, ObstacleRiskGrid obstacleRiskGrid,
            VehicleDynamics vehicleDynamics) : base(mapManager, startPos, goalPos, vehicleDynamics)
        {
            _obstacleRiskGrid = obstacleRiskGrid;
            _obstacleMap =  ObstacleMap.Initialize(MapManager,
                additionalObjects: new List<GameObject>(),
                cellScale: Vector3.one * GridResolution,
                margin: Vector3.one);
        }

        public override void PlanPath()
        {
            _grid = BuildGrid();

            //do A-Star in grid coordinates
            var start = _obstacleMap.WorldToCell(StartPos);
            var goal = _obstacleMap.WorldToCell(GoalPos);
            var cameFrom = new Dictionary<Vector3Int, Vector3Int>();
            var openSet = new List<Vector3Int>();
            openSet.Add(start);
            _grid.Remove(start);
            _grid.Add(start, new Vector2(0, (start - goal).magnitude));
            while (openSet.Count > 0)
            {
                var current = _grid
                    .Where(g => openSet.Contains(g.Key))
                    .OrderBy(g => g.Value.y)
                    .First()
                    .Key;
                if (current == goal) Path = ReconstructPath(cameFrom, current);
                openSet.Remove(current);
                var currentNeighbours = GetGridNeighbours(current, _grid, range:1);
                foreach (var n in currentNeighbours)
                {
                    var distanceToCurrent = (n - current).magnitude;
                    var tentativeGScore = _grid[current].x + distanceToCurrent;
                    if (tentativeGScore < _grid[n].x)
                    {
                        var distanceToGoal = (n - goal).magnitude;
                        cameFrom[n] = current;
                        _grid.Remove(n);
                        _grid.Add(n,
                            new Vector2(tentativeGScore, tentativeGScore + distanceToGoal));
                        if (!openSet.Contains(n)) openSet.Add(n);
                    }
                }
            }
        }

        public override Path GetPath()
        {
            return Path;
        }
        

        private Path ReconstructPath(Dictionary<Vector3Int, Vector3Int> cameFrom, Vector3Int current)
        {
            //go through path backwards
            var gridPath = new List<Vector3Int>();
            gridPath.Add(current);
            while (cameFrom.ContainsKey(current))
            {
                var next = cameFrom[current];
                gridPath.Add(next);
                current = next;
            }

            //translate grid path to world coordinates
            var waypoints = new List<State>();
            for (var i = gridPath.Count - 1; i >= 0; i--)
            {
                var cellToWorld = _obstacleMap.CellToWorld(gridPath[i]) + _obstacleMap.trueScale / 2;
                var waypoint = new State(cellToWorld.x, cellToWorld.z, 0, 1);
                waypoints.Add(waypoint);
            }

            return new Path(waypoints);
        }

        private List<Vector3Int> GetGridNeighbours(Vector3Int position, Dictionary<Vector3Int, Vector2> grid, int range)
        {
            //get 8 neighbourhood
            var neighbours = new List<Vector3Int>();
            for (var x = position.x - range; x <= position.x + range; x++)
            {
                for (var y = position.z - range; y <= position.z + range; y++)
                {
                    var neighbour = new Vector3Int(x, 0, y);
                    if (!grid.ContainsKey(neighbour)) continue;
                    if (x == position.x && y == position.z) continue;
                    neighbours.Add(neighbour);
                }
            }

            return neighbours;
        }

        private Dictionary<Vector3Int, Vector2> BuildGrid()
        {
            //build grid, <(position), (g_value, f_value)>
            var grid = new Dictionary<Vector3Int, Vector2>();
            foreach (var posEntity in this._obstacleMap.traversabilityPerCell)
            {
                var position = new Vector3Int(posEntity.Key.x, 0, posEntity.Key.y);
                var isValidCell = false;
                if (_obstacleRiskGrid == null)
                {
                    isValidCell = posEntity.Value == ObstacleMap.Traversability.Free;
                }
                else
                {
                    isValidCell = _obstacleRiskGrid.GetRiskAt(_obstacleMap.CellToWorld(position)) < 0.8f;
                }
                if (isValidCell)
                {
                    grid.Add(position, new Vector2(float.MaxValue, float.MaxValue));
                }
            }

            return grid;
        }
        
        public override void DebugDraw(int verbosity)
        {
            //draw grid
            foreach (var posEntity in _obstacleMap.traversabilityPerCell)
            {
                var position = new Vector3Int(posEntity.Key.x, 0, posEntity.Key.y);

                var worldCoord = _obstacleMap.CellToWorld(position) + _obstacleMap.trueScale / 2;
                worldCoord.y = 0;
                var cellToWorld = worldCoord + Vector3.up * 0.25f;

                var scale = Vector3.Scale(Vector3.one, _obstacleMap.trueScale);
                var gizmoSize = new Vector3(scale.x * 0.95f, 0.005f, scale.z * 0.95f);
                if (Path.Contains(worldCoord) && posEntity.Value == ObstacleMap.Traversability.Blocked)
                {
                    Gizmos.color = Color.blue;
                }
                else if (!_grid.ContainsKey(position))
                {
                    Gizmos.color = Color.red;
                }
                else if (posEntity.Value == ObstacleMap.Traversability.Free && !_grid.ContainsKey(position))
                {
                    Gizmos.color = Color.orange;
                }
                else if (Path.Contains(worldCoord))
                {
                    Gizmos.color = Color.yellow;
                }
                else
                {
                    Gizmos.color = Color.green;
                }

                Gizmos.DrawCube(cellToWorld, gizmoSize);
            }
        }
    }
}