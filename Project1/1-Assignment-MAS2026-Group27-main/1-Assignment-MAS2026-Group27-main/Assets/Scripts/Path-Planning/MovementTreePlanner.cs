using System;
using System.Collections.Generic;
using System.Linq;
using Scripts.Map;
using Unity.Mathematics;
using UnityEngine;
using static Scripts.Map.ObstacleMap;
using Random = System.Random;

namespace Path_Planning
{
    public class MovementTreePlanner: PathPlanner
    {
        private List<MovementPrimitive> _movementPrimitives = new();
        private MovementTree _movementTree;
        private ObstacleMap _obstacleMap;
        private ObstacleRiskGrid _obstacleRiskGrid;
        private State _startState;
        private Vector3 _goalPos;

        private int _maxGridXCoord;
        private int _minGridXCoord;
        private int _maxGridYCoord;
        private int _minGridYCoord;
        
        private AStarPlanner _aStarPlanner;
        //private static float _positionDistanceThreshold = 0.02f;
        private static float _headingDistanceThreshold = (float)Math.PI / 3f;
        private static float _goalThreshold = 5f;
        private static int _maxIterations = 35000; //10k ~ 3min planning time, 30k ~ 15min planning time

        private Path _path = new Path();

        public MovementTreePlanner(MapManager mapManager, Vector3 startPos, Vector3 goalPos,
            VehicleDynamics vehicleDynamics) : base(mapManager,
            startPos, goalPos, vehicleDynamics)
        {
            _startState = new State(startPos.x, startPos.z,(float)Math.PI / 2, 0);
            _startState.HeuristicValue = CalcHeuristicValue(_startState);
            _goalPos = goalPos;
            _obstacleMap =  ObstacleMap.Initialize(MapManager,
                additionalObjects: new List<GameObject>(),
                cellScale: Vector3.one,
                margin: Vector3.one);

            _obstacleRiskGrid = new ObstacleRiskGrid(_obstacleMap, _vehicleDynamics.VehicleRadius, ObstacleRiskDecayRate);

            _movementPrimitives = _vehicleDynamics.MovementPrimitives;

            _movementTree = new MovementTree(_startState);
            
            _maxGridXCoord = _obstacleMap.traversabilityPerCell.Keys
            .OrderBy(cell => cell.x).Last().x;
            _minGridXCoord = _obstacleMap.traversabilityPerCell.Keys
            .OrderBy(cell => cell.x).First().x;
            _maxGridYCoord = _obstacleMap.traversabilityPerCell.Keys
            .OrderBy(cell => cell.y).Last().y;
            _minGridYCoord = _obstacleMap.traversabilityPerCell.Keys
                .OrderBy(cell => cell.y).First().y;
        }

        
        public override void PlanPath()
        {
            Debug.Log("Starting movement tree path planning!");
            
            Debug.Log("Trying A-Star on default map");
            var time = Time.realtimeSinceStartup;
            _aStarPlanner = new AStarPlanner(MapManager, _startState.WorldPos(), _goalPos, _obstacleRiskGrid,
                _vehicleDynamics);
            _aStarPlanner.PlanPath();
            Debug.Log("A-Star took " + (Time.realtimeSinceStartup - time) + " seconds.");

            if (!_aStarPlanner.GetPath().IsInitialized())
            {
                Debug.Log("A-Star on the default map failed to produce a helper path.");
                var obstacleRiskGridAfterClosure =
                    new ObstacleRiskGrid(_obstacleMap, _vehicleDynamics.VehicleRadius, ObstacleRiskDecayRate);
                Debug.Log("Starting preparatory obstacle grid closure.");
                time = Time.realtimeSinceStartup;
                obstacleRiskGridAfterClosure.DeflateObstacles(2);
                Debug.Log("Finished preparatory obstacle grid deflation step. Took " +
                          (Time.realtimeSinceStartup - time) + " seconds.");
                time = Time.realtimeSinceStartup;
                obstacleRiskGridAfterClosure.InflateObstacles(2);
                Debug.Log("Finished preparatory obstacle grid inflation step. Took " +
                          (Time.realtimeSinceStartup - time) + " seconds.");
                _aStarPlanner = new AStarPlanner(MapManager, _startState.WorldPos(), _goalPos,
                    obstacleRiskGridAfterClosure,
                    _vehicleDynamics);
                time = Time.realtimeSinceStartup;
                _aStarPlanner.PlanPath();
                Debug.Log("Finished A-Star planning. Took " + (Time.realtimeSinceStartup - time) + " seconds.");
                if (!_aStarPlanner.GetPath().IsInitialized()) Debug.Log("A-Star was not successful");
                else Debug.Log("A-Star was successful");
            }
            else Debug.Log("A-Star was successful");
            
            time = Time.realtimeSinceStartup;
            BuildMovementTree();
            Debug.Log("Finished movement tree construction. Took " + (Time.realtimeSinceStartup - time) + " seconds.");
            ReconstructPath();
        }

        private void BuildMovementTree()
        {
            var aStarPath = _aStarPlanner.GetPath();
            var openSet = new SortedSet<State>();
            openSet.Add(_startState);
            var foundGoal = false;
            var i = 0;
            while (_maxIterations > i && openSet.Count > 0 && !foundGoal)
            {
                var state = GetNextState(openSet);

                foundGoal = ExploreMovements(state, openSet);
                
                openSet.Remove(state);
                if (i > 0 && i % 1000 == 0)
                    Debug.Log("Now in iteration " + i + ". Current state has distance to goal: " +
                              state.DistanceToPos(_goalPos) + ". " + (_maxIterations - i) + " iterations to go.");
                i++;
            }
            
            if (!foundGoal) Debug.Log("Reached maximum iterations: " + _maxIterations);
            else Debug.Log("Found path after " + i + " iterations!");
        }

        private State GetNextState(SortedSet<State> openSet)
        {
            var helperPath = _aStarPlanner.GetPath();
            var rnd = new Random();
            State state;
            if (rnd.Next(0, 100) > 80) //choose random current goal most of the time
            {
                if (helperPath.IsInitialized() && !helperPath.ReachedEnd())
                { 
                    state = openSet.OrderBy(s => helperPath.GetDistanceToPath(s)).First();
                }
                else
                {
                    state = openSet.OrderBy(s => s.DistanceToPos(_goalPos)).First();
                }
            }
            else
            {
                var currentGoalCell = new Vector3Int(rnd.Next(_minGridXCoord, _maxGridXCoord), 0,
                    rnd.Next(_minGridYCoord, _maxGridYCoord));
                var randomPos = _obstacleMap.CellToWorld(currentGoalCell);
                state = openSet.OrderBy(s => s.DistanceToPos(randomPos)).First();
            }

            return state;
        }

        private bool ExploreMovements(State state, SortedSet<State> openSet)
        {
            var helperPath = _aStarPlanner.GetPath();
            var foundGoal = false;
            foreach (var movement in _movementPrimitives)
            {
                var newState = _vehicleDynamics.Move(state, movement);
                if (_obstacleRiskGrid.GetRiskAt(newState.WorldPos()) > 0.45f) continue;
                if (newState.Velocity > _vehicleDynamics.MaxVelocity) continue;
                if (_movementTree.GetNodes().Any(n =>
                        n.DistanceToPos(newState) < _vehicleDynamics.PlanningNodeDistanceThreshold // <-- modified
                        &&
                        n.DistanceToHeading(newState) < _headingDistanceThreshold
                    )) continue;
                newState.HeuristicValue = CalcHeuristicValue(newState);
                _movementTree.Set(state, newState, movement);
                if (helperPath.IsInitialized() && !helperPath.ReachedEnd() &&
                    newState.DistanceToPos(helperPath.GetClosest(newState)) < 4*_goalThreshold)
                {
                    helperPath.JumpTo(helperPath.GetClosest(newState));
                }
                foundGoal = foundGoal || newState.DistanceToPos(_goalPos) < _goalThreshold;
                openSet.Add(newState);
            }

            return foundGoal;
        }

        private float CalcHeuristicValue(State state)
        {
            return Vector3.Distance(_goalPos, state.WorldPos());
        }

        private void ReconstructPath()
        {
            var wayPoints = new List<State>();
            var currentState = _movementTree.GetClosestNodeTo(_goalPos);
            if (currentState.DistanceToPos(_goalPos) > _goalThreshold) return;
            while (currentState != _startState)
            {
                wayPoints.Add(currentState);
                currentState = _movementTree.GetParent(currentState);
            }
            wayPoints.Add(_startState);
            wayPoints.Reverse();
            _path = new Path(wayPoints);
        }

        public override Path GetPath()
        {
            return _path;
        }

        public ObstacleRiskGrid GetObstacleRiskGrid()
        {
            return _obstacleRiskGrid;
        }

        public override void DebugDraw(int verbosity)
        {
            GetPath().DebugDraw();
            switch (verbosity)
            {
                case 1:
                    _movementTree.DebugDraw();
                    break;
                case 2:
                    _aStarPlanner.DebugDraw(verbosity);
                    break;
                case 3:
                    _obstacleRiskGrid.DebugDraw();
                    break;
            }
        }
    }
}
