using System.Collections.Generic;
using System.Linq;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Path_Planning;
using Scripts.Vehicle;
using UnityEngine;

namespace Path_Execution
{
    public class SimpleDronePathFollower: PathExecutor
    {
        private new DroneController Controller;
        private Rigidbody _rb;
        
        private List<Vector3> directions;
        
        // Tunable parameters
        private float _waypointReachedDistance = 5f; 
        private int _lookAheadDistance = 3; 
        private float _maxSpeed = 3f; 
        private float _obstacleAvoidanceStrength = 4f;
        private float _obstacleDetectionRange = 5f; 
        private float _dampingFactor = 0.5f;

        private float _timeSinceLastMove = 0f;
        
        public SimpleDronePathFollower(Path path, DroneController drone) : base(path, drone)
        {
            _rb = drone.GetComponent<Rigidbody>();
            Controller = drone;

            directions = new List<Vector3>();
            for (int i = 0; i < 36; i++)
            {
                directions.Add(Quaternion.AngleAxis(i * 10f, Vector3.up) * Vector3.forward);
            }
        }

        public override void Step()
        {
            if (_rb.linearVelocity.magnitude > 0.5f) _timeSinceLastMove = Time.realtimeSinceStartup;
            if (Time.realtimeSinceStartup - _timeSinceLastMove > 5f)
            {
                Debug.Log("Lowering obstacle avoidance strength:" + _obstacleAvoidanceStrength);
                if (_obstacleAvoidanceStrength > 0) _obstacleAvoidanceStrength -= 0.02f;
            }
            if (Path.IsInitialized() && !Path.ReachedEnd())
            {
                // Get target position with look-ahead for smoother movement
                Vector3 targetPos = Path.PeekWayPoint(_lookAheadDistance).WorldPos();
                
                var currentX = Controller.transform.position.x;
                var currentY = Controller.transform.position.z;
                var deltaX = targetPos.x - currentX;
                var deltaY = targetPos.z - currentY;
                
                var distanceToTarget = new Vector2(deltaX, deltaY).magnitude;
                
                if (distanceToTarget > 0.1f)
                {
                    deltaX /= distanceToTarget;
                    deltaY /= distanceToTarget;
                }
                
                var adjustment = GetInputAdjustmentRaycast();
                deltaX += adjustment.x;
                deltaY += adjustment.y;
                
                float magnitude = Mathf.Sqrt(deltaX * deltaX + deltaY * deltaY);
                if (magnitude > 0.1f)
                {
                    deltaX /= magnitude;
                    deltaY /= magnitude;
                }
                
                float desiredSpeed = Mathf.Min(distanceToTarget * 0.5f, _maxSpeed);
                deltaX *= desiredSpeed;
                deltaY *= desiredSpeed;
                
                Vector3 currentVelocity = _rb.linearVelocity;
                deltaX -= currentVelocity.x * _dampingFactor;
                deltaY -= currentVelocity.z * _dampingFactor;
                
                deltaX /= Controller.max_acceleration;
                deltaY /= Controller.max_acceleration;
                
                deltaX = Mathf.Clamp(deltaX, -1f, 1f);
                deltaY = Mathf.Clamp(deltaY, -1f, 1f);
                
                Controller.Move(deltaX, deltaY);

                var state = new State(Controller.transform.position.x, Controller.transform.position.z, 0, 0);
                var currentGoal = Path.GetClosest(state);
                if (currentGoal.DistanceToPos(Controller.transform.position) < _waypointReachedDistance) 
                {
                    Path.JumpTo(state);
                }
            }
        }

        private Vector2 GetInputAdjustmentRaycast()
        {
            var maxRange = 500f;
            var adjustment3D = Vector3.zero;
            
            foreach (var direction in directions)
            {
                float obstacleDistance;
                if (ObstacleClose(direction, maxRange, out obstacleDistance))
                {
                    float repulsionStrength = Mathf.Clamp01(1f - (obstacleDistance / _obstacleDetectionRange));
                    adjustment3D -= direction * _obstacleAvoidanceStrength * repulsionStrength;
                }
            }
            
            return new Vector2(adjustment3D.x, adjustment3D.z);
        }
        
        private bool ObstacleClose(Vector3 direction, float maxRange)
        {
            float distance;
            return ObstacleClose(direction, maxRange, out distance);
        }
        
        private bool ObstacleClose(Vector3 direction, float maxRange, out float distance)
        {
            var transformedDirection = Controller.transform.TransformDirection(direction);
            RaycastHit hit;
            
            if (Physics.Raycast(Controller.transform.position + Controller.transform.up, transformedDirection, out hit, maxRange))
            {
                distance = hit.distance;
                return hit.distance < _obstacleDetectionRange;
            }
            
            distance = maxRange;
            return false;
        }
        
        
        public override void DebugDraw()
        {
            foreach (var direction in directions)
            {
                var transformedDirection = Controller.transform.TransformDirection(direction);
                Gizmos.color = Color.aquamarine;
                if (ObstacleClose(direction, 500f)) Gizmos.color = Color.red;
                Debug.DrawRay(Controller.transform.position + Controller.transform.up, transformedDirection * 4, Gizmos.color);
            }
            
            // Draw current target
            if (Path.IsInitialized() && !Path.ReachedEnd())
            {
                Debug.DrawLine(Controller.transform.position, Path.PeekWayPoint(_lookAheadDistance).WorldPos(), Color.yellow);
            }
        }
    }
}