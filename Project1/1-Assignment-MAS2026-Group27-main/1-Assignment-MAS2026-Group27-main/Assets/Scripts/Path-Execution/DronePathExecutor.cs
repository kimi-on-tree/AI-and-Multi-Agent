using UnityEngine;
using Path_Planning;
using Scripts.Vehicle;

namespace Path_Execution
{
    public class DronePathExecutor
    {
        private Path _path;
        private DroneController _controller;
        private Rigidbody _rb;

        private const int LookAheadIndex = 5;

        private const float kCT_P = 4.0f;   
        private const float kCT_D = 1.0f;   

        private const float kV_P = 3.0f;  
        private const float kAT_P = 0.5f;  

        private const float MaxPassDistanceSqr = 20.0f;
        private const float MinSnapDistanceSqr = 0.5f * 0.5f;

        public DronePathExecutor(Path path, DroneController controller)
        {
            _path = path;
            _controller = controller;
            _rb = _controller.GetComponent<Rigidbody>();
        }

        public void Step()
        {
            if (_path == null || !_path.IsInitialized() || _path.ReachedEnd())
            {
                _controller.Move(0, 0);
                return;
            }

            Vector3 currentPos = _controller.transform.position;
            Vector3 currentVel = _rb.linearVelocity;

            while (!_path.ReachedEnd())
            {
                State currentTargetNode = _path.GetCurrentWayPoint();
                Vector3 targetPos = currentTargetNode.WorldPos();

                Vector3 toDroneVector = currentPos - targetPos;
                toDroneVector.y = 0;

                float heading = currentTargetNode.Heading;
                Vector3 targetForward = new Vector3(Mathf.Cos(heading), 0, Mathf.Sin(heading));

                float dot = Vector3.Dot(toDroneVector, targetForward);
                float distSqr = toDroneVector.sqrMagnitude;

                bool passedPlane = dot >= 0 && distSqr < MaxPassDistanceSqr;
                bool tooClose = distSqr < MinSnapDistanceSqr;

                if (passedPlane || tooClose)
                {
                    _path.Step();
                }
                else
                {
                    break;
                }
            }

            if (_path.ReachedEnd())
            {
                _controller.Move(0, 0);
                return;
            }

            State refState = _path.PeekWayPoint(LookAheadIndex);
            if (refState == null) refState = _path.GetWayPoint(_path.GetLength() - 1);

            Vector3 pRef = refState.WorldPos();
            float vRefSpeed = refState.Velocity;

            float hRef = refState.Heading;
            Vector3 tHat = new Vector3(Mathf.Cos(hRef), 0, Mathf.Sin(hRef));
            Vector3 nHat = new Vector3(-tHat.z, 0, tHat.x);

            Vector3 pError = pRef - currentPos;
            pError.y = 0;
            currentVel.y = 0;

            float eCT = Vector3.Dot(pError, nHat); 
            float eAT = Vector3.Dot(pError, tHat); 

            float vCT = Vector3.Dot(currentVel, nHat); 
            float vAT = Vector3.Dot(currentVel, tHat); 

            float aCT = (kCT_P * eCT) - (kCT_D * vCT);
            float aAT = kV_P * (vRefSpeed - vAT) + (kAT_P * eAT);

            Vector3 accelerationCmd = (aAT * tHat) + (aCT * nHat);

            float maxAccel = _controller.max_acceleration;

            if (accelerationCmd.magnitude > maxAccel)
            {
                accelerationCmd = accelerationCmd.normalized * maxAccel;
            }

            float h_input = Mathf.Clamp(accelerationCmd.x / maxAccel, -1f, 1f);
            float v_input = Mathf.Clamp(accelerationCmd.z / maxAccel, -1f, 1f);

            _controller.Move(h_input, v_input);
        }
        public Path GetPath() => _path;
    }
}