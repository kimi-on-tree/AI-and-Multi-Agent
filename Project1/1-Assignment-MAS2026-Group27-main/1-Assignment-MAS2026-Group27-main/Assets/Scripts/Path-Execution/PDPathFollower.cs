using System.Collections.Generic;
using System.Linq;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using UnityEngine;
using Scripts.Map;
using Path_Planning;

namespace Path_Execution
{
    public class PDPathFollower : PathExecutor
    {
        [Header("Lateral Control (Steering)")]
        public float Kp_Lat = 0.6f;
        public float Kd_Lat = 0.3f;
        public float Kp_Heading = 1.2f;

        [Header("Longitudinal Control (Speed)")]
        public float Kp_Speed = 1.0f;
        public float Kd_Speed = 0.1f;
        public float MaxAccel = 0.8f;
        public float MaxBrake = 0.8f;
        public float BrakeLookAheadDist = 4.0f;
        public float SpeedOffset = 0.0f; 

        [Header("Sharp Turn Logic")]
        public int SharpTurnLookAheadIndex = 4;
        public float SharpTurnThresholdAngle = 60f;
        public float SharpTurnSpeedLimit = 8.0f;

        [Header("Settings")]
        public float MaxSteerAngle = 30f;
        public bool InvertSteering = false;

        private float FrontAxleOffset = 2.0f;

        private Vector3 _currentClosestPoint;
        private Vector3 _debugTargetPos;
        private float _lastCrossTrackError = 0f;
        private float _lastSpeedError = 0f;
        private float _lastCollision = 0f;
        
        private new CarController Controller;

        private Rigidbody _rb;

        private Dictionary<Vector3, float> directions;

        public PDPathFollower(Path path, CarController car) : base(path, car)
        {
            _rb = car.GetComponent<Rigidbody>();

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
            if (Path == null || !Path.IsInitialized() || _rb == null) return;

            var collision = CollisionHandle();
            if (collision || ((Time.realtimeSinceStartup) - _lastCollision) < 0.8f)
            {
                if (collision) _lastCollision = Time.realtimeSinceStartup;
                Controller.Move(0, -1f, -1, 0);
                return;
            }

            Vector3 frontAxlePos = Controller.transform.position + Controller.transform.forward * FrontAxleOffset;

            while (!Path.ReachedEnd())
            {
                State currentWP = Path.GetCurrentWayPoint();
                Vector3 wpToCar = frontAxlePos - currentWP.WorldPos();
                float h = currentWP.Heading;
                Vector3 wpForward = new Vector3(Mathf.Cos(h), 0, Mathf.Sin(h));

                if (Vector3.Dot(wpToCar, wpForward) > 0)
                {
                    Path.Step();
                }
                else
                {
                    break;
                }
            }

            State currentTarget = Path.GetCurrentWayPoint();
            State prevWP = (Path.GetCurrentIdx() > 0) ? Path.PeekWayPoint(-1) : currentTarget;

            Vector3 projectedPos = GetProjectionPoint(prevWP.WorldPos(), currentTarget.WorldPos(), frontAxlePos);
            _currentClosestPoint = projectedPos;
            _debugTargetPos = currentTarget.WorldPos();
            float currentSpeed = 0f;
            if (Controller.GetComponent<Rigidbody>() != null)
            {
                currentSpeed = Vector3.Dot(Controller.transform.forward, Controller.GetComponent<Rigidbody>().linearVelocity);
            }

            float segmentLen = Vector3.Distance(prevWP.WorldPos(), currentTarget.WorldPos());
            float distFromPrev = Vector3.Distance(prevWP.WorldPos(), projectedPos);
            float t = (segmentLen > 0.001f) ? Mathf.Clamp01(distFromPrev / segmentLen) : 1.0f;

            float baseTargetSpeed = Mathf.Lerp(prevWP.Velocity, currentTarget.Velocity, t);
            float finalTargetSpeed = baseTargetSpeed + SpeedOffset;

            State futureState = GetLookAheadState(BrakeLookAheadDist);
            if (Mathf.Abs(futureState.Velocity) < Mathf.Abs(finalTargetSpeed))
            {
                finalTargetSpeed = futureState.Velocity + SpeedOffset;
            }

            State futureTurnWP = Path.PeekWayPoint(SharpTurnLookAheadIndex);
            if (futureTurnWP != null)
            {
                float currentHeadingDeg = currentTarget.Heading * Mathf.Rad2Deg;
                float futureHeadingDeg = futureTurnWP.Heading * Mathf.Rad2Deg;
                float angleDiff = Mathf.Abs(Mathf.DeltaAngle(currentHeadingDeg, futureHeadingDeg));
                if (angleDiff > SharpTurnThresholdAngle)
                {
                    float limit = SharpTurnSpeedLimit;
                    if (finalTargetSpeed < 0) limit = -limit;

                    if (Mathf.Abs(limit) < Mathf.Abs(finalTargetSpeed))
                    {
                        finalTargetSpeed = limit;
                    }
                }
            }

            if (Mathf.Abs(finalTargetSpeed) < 0.1f && !Path.ReachedEnd())
            {
                finalTargetSpeed = finalTargetSpeed >= 0 ? 1.0f : -1.0f;
            }

            float speedError = finalTargetSpeed - currentSpeed;
            float dt = Time.deltaTime;
            if (dt <= 0.0001f) dt = 0.02f;

            float speedErrorRate = (speedError - _lastSpeedError) / dt;
            _lastSpeedError = speedError;

            float speedOutput = (Kp_Speed * speedError) + (Kd_Speed * speedErrorRate);

            float throttle = 0f;
            float brake = 0f;

            if (finalTargetSpeed >= 0) 
            {
                if (speedOutput > 0)
                {
                    throttle = Mathf.Clamp(speedOutput, 0f, MaxAccel);
                    brake = 0f;
                }
                else
                {
                    throttle = 0f;
                    brake = Mathf.Clamp(-speedOutput, 0f, MaxBrake);
                }
            }
            else 
            {
                if (speedOutput < 0)
                {
                    throttle = 0f;
                    brake = Mathf.Clamp(speedOutput, -MaxBrake, 0f); 
                }
                else
                {
                    throttle = 0f;
                    brake = Mathf.Clamp(speedOutput, 0f, MaxBrake);
                }
            }

            float steerInput = CalculateCoupledSteering(currentTarget, dt, frontAxlePos);

            var steeringAdjustment = GetSteeringAdjustmentRaycast(currentSpeed);
            steerInput += steeringAdjustment;

            Controller.Move(steerInput, throttle, brake, 0f);
        }

        private float CalculateCoupledSteering(State targetState, float dt, Vector3 controlPos)
        {
            Vector3 offset = targetState.WorldPos() - controlPos;

            // CTE PD
            float crossTrackError = Vector3.Dot(offset, Controller.transform.right);
            float errorRate = (crossTrackError - _lastCrossTrackError) / dt;
            _lastCrossTrackError = crossTrackError;
            float distTerm = (Kp_Lat * crossTrackError) + (Kd_Lat * errorRate);

            // Heading PD
            float targetHeadingRad = targetState.Heading;
            Vector3 targetForward = new Vector3(Mathf.Cos(targetHeadingRad), 0, Mathf.Sin(targetHeadingRad));
            float headingErrorAngle = Vector3.SignedAngle(Controller.transform.forward, targetForward, Vector3.up);

            if (Mathf.Abs(headingErrorAngle) < 0.5f) headingErrorAngle = 0f;

            float headingTerm = Kp_Heading * (headingErrorAngle / MaxSteerAngle);

            float combinedSteer = distTerm + headingTerm;
            float finalSteer = Mathf.Clamp(combinedSteer, -1f, 1f);
            if (InvertSteering) finalSteer = -finalSteer;
            return finalSteer;
        }

        private Vector3 GetProjectionPoint(Vector3 start, Vector3 end, Vector3 point)
        {
            Vector3 lineDir = end - start;
            float len = lineDir.magnitude;
            if (len < 0.001f) return start;
            lineDir.Normalize();
            Vector3 v = point - start;
            float d = Vector3.Dot(v, lineDir);
            d = Mathf.Clamp(d, 0f, len);
            return start + lineDir * d;
        }

        private State GetLookAheadState(float targetDist)
        {
            if (Path.ReachedEnd()) return Path.GetCurrentWayPoint();
            int offset = 0;
            State candidate = Path.GetCurrentWayPoint();
            while (offset < 100)
            {
                if (Path.GetCurrentIdx() + offset >= Path.GetLength() - 1) return Path.PeekWayPoint(offset);
                candidate = Path.PeekWayPoint(offset);
                if (Vector3.Distance(Controller.transform.position, candidate.WorldPos()) >= targetDist) return candidate;
                offset++;
            }
            return candidate;
        }

        private bool CollisionHandle()
        {
            RaycastHit hit;
            var collison = false;
            float maxRange = 3f;
            if (Physics.Raycast(Controller.transform.position + Controller.transform.up + Controller.transform.TransformDirection(Vector3.right * 1.2f), Controller.transform.TransformDirection(Vector3.forward), out hit, maxRange)) collison = true;
            if (Physics.Raycast(Controller.transform.position + Controller.transform.up, Controller.transform.TransformDirection(Vector3.forward), out hit, maxRange)) collison = true;
            if (Physics.Raycast(Controller.transform.position + Controller.transform.up + Controller.transform.TransformDirection(Vector3.left * 1.2f), Controller.transform.TransformDirection(Vector3.forward), out hit, 0.5f)) collison = true;
            return collison;
        }

        private float GetSteeringAdjustmentRaycast(float currentSpeed)
        {
            float maxRange = 500f;
            return directions.Keys.Where(direction => ObstacleClose(direction, currentSpeed, maxRange)).Sum(direction => directions[direction]);
        }

        private bool ObstacleClose(Vector3 direction, float currentSpeed, float maxRange)
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
            if (Controller == null) return;
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(_currentClosestPoint, 0.5f);
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(_debugTargetPos, 0.5f);
        }
    }
}