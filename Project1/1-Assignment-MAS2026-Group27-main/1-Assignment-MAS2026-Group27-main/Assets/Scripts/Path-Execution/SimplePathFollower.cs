using System.Collections.Generic;
using System.Linq;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using UnityEngine;

namespace Path_Execution
{
    //Simple binary path follower
    public class SimplePathFollower: PathExecutor
    {

        private float Accel = 1f;
        
        private Dictionary<Vector3, float> directions;
        private float _lastCollision = 0f;
        private new CarController Controller;
        public SimplePathFollower(Path path, CarController car): base(path, car)
        {
            directions = new Dictionary<Vector3, float>
            {
                {Quaternion.AngleAxis(10, Vector3.up) * Vector3.forward, -0.2f},
                {Quaternion.AngleAxis(-10, Vector3.up) * Vector3.forward, 0.2f},
                {Quaternion.AngleAxis(20, Vector3.up) * Vector3.forward, -0.1f},
                {Quaternion.AngleAxis(-20, Vector3.up) * Vector3.forward, 0.1f},
                {Quaternion.AngleAxis(30, Vector3.up) * Vector3.forward, -0.05f},
                {Quaternion.AngleAxis(-30, Vector3.up) * Vector3.forward, 0.05f}
            };
            Controller = car;
        }

        public override void Step()
        {
            var collision = CollisionHandle();
            if (collision || ((Time.realtimeSinceStartup) - _lastCollision) < 0.5f)
            {
                if (collision) _lastCollision = Time.realtimeSinceStartup;
                Controller.Move(0, -1f, -1, 0);
                return;
            }
            if (!Path.IsInitialized()) return;
            var currentGoalGlobal = Path.GetCurrentWayPoint();
            var currentGoalLocal = Controller.transform.InverseTransformPoint(currentGoalGlobal.WorldPos());
            var distanceToGoal = (currentGoalLocal).magnitude;
            float steering = 0.0f;
            float currentSpeed = 0f;
            if (Controller.GetComponent<Rigidbody>() != null)
            {
                currentSpeed = Vector3.Dot(Controller.transform.forward, Controller.GetComponent<Rigidbody>().linearVelocity);
            }
            
            Debug.Log("Distance to goal: " + distanceToGoal);
            if (distanceToGoal < 5)
            {
                if (Path.ReachedEnd())
                {
                    Controller.Move(0f, 0f, 1f, 1f);
                    return;
                }
                Path.Step();
            }

            if (currentGoalLocal[0] > 2)
            {
                Debug.Log("RIGHT!");
                steering = 1f;
            }
            else if (currentGoalLocal[0] < -2)
            {
                Debug.Log("LEFT!");
                steering = -1f;
            }
            else if (currentGoalLocal[2] < 0)
            {
                Debug.Log("BEHIND!");
                steering = 1f;
            }
            else
            {
                Debug.Log("FORWARD!");
                steering = 0f;
            }
            
            //adjust steering using ray cast
            var steeringAdjustment = GetSteeringAdjustmentRaycast();
            Debug.Log("Steering adjustment: " + steeringAdjustment);
            steering += steeringAdjustment;
            var v = (currentGoalGlobal.Velocity - currentSpeed) * 5;
            Controller.Move(steering, v, v, 0f);
        }

        private bool CollisionHandle()
        {
            RaycastHit hit;
            var collison = false;
            float maxRange = 3f;
            if (Physics.Raycast(
                    Controller.transform.position + Controller.transform.up + Controller.transform.TransformDirection(Vector3.right * 1.2f),
                    Controller.transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Debug.Log("Right forward hit"!);
                collison = true;
            } 
            if (Physics.Raycast(
                    Controller.transform.position + Controller.transform.up,
                    Controller.transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Debug.Log("Middle forward hit"!);
                collison = true;
            }
            if (Physics.Raycast(
                    Controller.transform.position + Controller.transform.up + Controller.transform.TransformDirection(Vector3.left * 1.2f),
                    Controller.transform.TransformDirection(Vector3.forward), out hit, 0.5f))
            {
                Debug.Log("Left forward hit"!);
                collison = true;
            }
            return collison;
        }

        private float GetSteeringAdjustmentRaycast()
        {
            RaycastHit hit;
            var maxRange = 500f;
            return directions.Keys
                .Where(direction => ObstacleClose(direction, maxRange))
                .Sum(direction => directions[direction]);
        }

        private bool ObstacleClose(Vector3 direction, float maxRange)
        {
            var transformedDirection = Controller.transform.TransformDirection(direction);
            RaycastHit hit;
            if (Physics.Raycast(Controller.transform.position + Controller.transform.up, transformedDirection, out hit, maxRange))
            {
                if (hit.distance < 8) return true;
            }
            return false;
        }
        
        
        public override void DebugDraw()
        {
            if (Path != null && Controller != null)
            {
                foreach (var direction in directions.Keys)
                {
                    var transformedDirection = Controller.transform.TransformDirection(direction);
                    RaycastHit hit;
                    float maxRange = 500f;
                    if (Physics.Raycast(Controller.transform.position + Controller.transform.up, transformedDirection, out hit, maxRange))
                    {
                        Vector3 closestObstacleInFront = transformedDirection * hit.distance;
                        Gizmos.color = Color.yellow;
                        if (ObstacleClose(direction, maxRange)) Gizmos.color = Color.red;
                        Gizmos.DrawRay(Controller.transform.position, closestObstacleInFront);
                    }
                }
            }
        }
        
    }

}