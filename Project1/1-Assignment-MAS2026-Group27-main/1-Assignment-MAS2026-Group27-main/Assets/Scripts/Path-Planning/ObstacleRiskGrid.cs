using System;
using System.Collections.Generic;
using Scripts.Map;
using UnityEngine;
using static Scripts.Map.ObstacleMap;

namespace Path_Planning
{
    public class ObstacleRiskGrid
    {
        private float _carRadius;
        private float _decayRate;
        private ObstacleMap _obstacleMap;
        private Dictionary<Vector2Int, double> _grid = new();
        private static Vector3 _maxRiskColorRgb = new Vector3(244, 60, 60);
        private static Vector3 _minRiskColorRgb = new Vector3(110, 185, 240);

        public ObstacleRiskGrid(ObstacleMap obstacleMap, float carRadius, float decayRate)
        {
            _carRadius = carRadius;
            _decayRate = decayRate;
            _obstacleMap = obstacleMap;
            this.CalcGridValues();
        }

        private void CalcGridValues()
        {
            foreach (var pos in _obstacleMap.traversabilityPerCell.Keys)
            {
                var obstacleRisk = 1.0d;
                if (_obstacleMap.traversabilityPerCell[pos] != ObstacleMap.Traversability.Blocked)
                {
                    var distanceToNearestBlocked = GetDistanceToNearestBlocked(pos);
                    if (distanceToNearestBlocked > _carRadius)
                    {
                        obstacleRisk = Math.Exp(-_decayRate * Math.Pow(distanceToNearestBlocked - _carRadius, 2));
                    }
                }
                _grid[pos] = obstacleRisk;
            }
        }

        public void InflateObstacles(int range)
        {
            var blocked = new List<Vector2Int>();
            foreach (var cell in _obstacleMap.traversabilityPerCell.Keys)
            {
                var neighbours = GetCellNeighbours(cell, range);
                if (_obstacleMap.traversabilityPerCell[cell] == ObstacleMap.Traversability.Blocked)
                {
                    blocked.AddRange(neighbours);
                }
            }

            foreach (var cell in blocked)
            {
                _obstacleMap.traversabilityPerCell[cell] = ObstacleMap.Traversability.Blocked;
            }
            CalcGridValues();
        }

        public void DeflateObstacles(int range)
        {
            var free = new List<Vector2Int>();
            foreach (var cell in _obstacleMap.traversabilityPerCell.Keys)
            {
                var neighbours = GetCellNeighbours(cell, range);
                if (_obstacleMap.traversabilityPerCell[cell] == ObstacleMap.Traversability.Free)
                {
                    free.AddRange(neighbours);
                }
            }

            foreach (var cell in free)
            {
                _obstacleMap.traversabilityPerCell[cell] = ObstacleMap.Traversability.Free;
            }
            CalcGridValues();
        }
        
        private List<Vector2Int> GetCellNeighbours(Vector2Int cell, int range)
        {
            var neighbours = new List<Vector2Int>();
            for (var x = cell.x - range; x <= cell.x + range; x++)
            {
                for (var y = cell.y - range; y <= cell.y + range; y++)
                {
                    var neighbour = new Vector2Int(x, y);
                    if (x == cell.x && y == cell.y) continue;
                    if (!_obstacleMap.traversabilityPerCell.ContainsKey(neighbour)) continue;
                    neighbours.Add(neighbour);
                }
            }

            return neighbours;
        }

        public double GetRiskAt(Vector2Int pos)
        {
            //get risk at grid position
            if (_grid.ContainsKey(pos))
            {
                return _grid[pos];
            }
            else
            {
                return -1;
            }
        }

        public double GetRiskAt(Vector3 pos)
        {
            //get risk at world position
            var cellCoords = _obstacleMap.WorldToCell(pos);
            return GetRiskAt(new Vector2Int(cellCoords.x, cellCoords.z));
        }
        
        private float GetDistanceToNearestBlocked(Vector2Int pos)
        {
            var range = 1;
            var searching = true;
            var distance = float.MaxValue;
            while (searching)
            {
                for (int x = pos.x - range; x < pos.x + range; x++)
                {
                    for (int y = pos.y - range; y <pos.y + range; y++)
                    {
                        if (x == pos.x && y == pos.y) continue;
                        var currentPos = new Vector2Int(x, y);
                        if (_obstacleMap.traversabilityPerCell.ContainsKey(currentPos))
                        {
                            if (_obstacleMap.traversabilityPerCell[currentPos] == Traversability.Blocked)
                            {
                                searching = false;
                                var currentDistance = (pos - currentPos).magnitude;
                                if (currentDistance < distance) distance = currentDistance;
                            }
                        }
                    }
                }
                range++;
            }

            return distance;
        }

        public void DebugDraw()
        {
            foreach (var posEntity in _obstacleMap.traversabilityPerCell)
            {
                var position = new Vector3Int(posEntity.Key.x, 0, posEntity.Key.y);

                var worldCoord = _obstacleMap.CellToWorld(position) + _obstacleMap.trueScale / 2;
                worldCoord.y = 0;
                var cellToWorld = worldCoord + Vector3.up * 0.25f;

                var scale = Vector3.Scale(Vector3.one, _obstacleMap.trueScale);
                var gizmoSize = new Vector3(scale.x * 0.95f, 0.005f, scale.z * 0.95f);

                var colorRgb = (_maxRiskColorRgb * (float)GetRiskAt(posEntity.Key)
                                + _minRiskColorRgb * (float)(1 - GetRiskAt(posEntity.Key))) / 255;

                Gizmos.color = new Color(
                    (float)Math.Round(colorRgb.x), 
                    (float)Math.Round(colorRgb.y), 
                    (float)Math.Round(colorRgb.z), 
                    255.0f);

                Gizmos.DrawCube(cellToWorld, gizmoSize);
            }
        }
    }
}