using System.Collections.Generic;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using UnityEngine;
using Scripts.Map;

namespace Path_Execution
{
    public class MPCPathFollower : PathExecutor
    {
        [Header("MPC Parameters")]
        public float HorizonTime = 3.0f;
        public int Steps = 15;
        public int NumSamples = 150;

        [Header("Vehicle Model")]
        public float WheelBase = 1.87f;
        public float VehicleWidth = 2.0f;

        public float MaxSteerLowSpeed = 45f;
        public float MaxSteerHighSpeed = 15f;
        public float LowSpeedThresh = 5.0f;
        public float HighSpeedThresh = 15.0f;

        public float MaxAccel = 1.0f;
        public float MaxBrake = 1.0f;

        [Header("Weights")]
        public float TargetSpeed = 2.0f;
        public float WeightDist = 8.0f;
        public float WeightSpeed = 1.0f;
        public float WeightObstacle = 10000f;
        public float WeightHeading = 10.0f;
        public float WeightSteerChange = 0.5f;

        private float _dt;
        private float[] _nominalSteer;
        private float[] _nominalAccel;
        private ObstacleMap _obstacleMap;

        private float _lastSteerOutput = 0f;

        private List<Vector3>[] _debugTrajectories;
        private Vector3[] _bestTrajectoryPoints;
        private Vector3 _currentDebugTarget;

        private new CarController Controller;

        public MPCPathFollower(Path path, CarController car, ObstacleMap map) : base(path, car)
        {
            _obstacleMap = map;
            InitializeMPC();
            Controller = car;
        }

        private void InitializeMPC()
        {
            _dt = HorizonTime / Steps;
            _nominalSteer = new float[Steps];
            _nominalAccel = new float[Steps];
            _bestTrajectoryPoints = new Vector3[Steps];

            for (int i = 0; i < Steps; i++)
            {
                _nominalSteer[i] = 0f;
                _nominalAccel[i] = 0.2f;
            }
        }

        public override void Step()
        {
            if (Path == null || !Path.IsInitialized()) return;

            float distToCurrent = Vector3.Distance(Controller.transform.position, Path.GetCurrentWayPoint().WorldPos());
            if (distToCurrent < 1.0f)
            {
                Path.StepUntilFar(Controller.transform.position, 1.0f);
            }

            State currentState = GetVehicleState();

            Optimize(currentState);

            float rawSteer = _nominalSteer[0];
            float rawAccel = _nominalAccel[0];

            float lerpFactor = (currentState.v < 3.0f) ? 0.6f : 0.2f;
            float smoothSteer = Mathf.Lerp(_lastSteerOutput, rawSteer, lerpFactor);
            _lastSteerOutput = smoothSteer;

            float throttle = (rawAccel > 0) ? rawAccel * MaxAccel : 0;
            float brake = (rawAccel < 0) ? -rawAccel * MaxBrake : 0;

            Controller.Move(-smoothSteer, throttle, brake, 0f);

            for (int i = 0; i < Steps - 1; i++)
            {
                _nominalSteer[i] = _nominalSteer[i + 1];
                _nominalAccel[i] = _nominalAccel[i + 1];
            }
            _nominalSteer[Steps - 1] = _nominalSteer[Steps - 2];
            _nominalAccel[Steps - 1] = 0f;
        }

        public override void DebugDraw()
        {
            throw new System.NotImplementedException();
        }

        private float GetDynamicMaxSteer(float speed)
        {
            float s = Mathf.Abs(speed);
            if (s < LowSpeedThresh) return MaxSteerLowSpeed;
            if (s > HighSpeedThresh) return MaxSteerHighSpeed;

            float t = (s - LowSpeedThresh) / (HighSpeedThresh - LowSpeedThresh);
            return Mathf.Lerp(MaxSteerLowSpeed, MaxSteerHighSpeed, t);
        }

        private void Optimize(State s0)
        {
            if (_debugTrajectories == null || _debugTrajectories.Length != NumSamples)
                _debugTrajectories = new List<Vector3>[NumSamples];

            float[] sampleCosts = new float[NumSamples];
            float[][] sampleSteers = new float[NumSamples][];
            float[][] sampleAccels = new float[NumSamples][];

            float minCost = float.MaxValue;

            for (int k = 0; k < NumSamples; k++)
            {
                sampleSteers[k] = new float[Steps];
                sampleAccels[k] = new float[Steps];
                _debugTrajectories[k] = new List<Vector3>();

                float totalCost = 0f;
                State simState = s0;
                float prevSteer = _nominalSteer[0];

                for (int t = 0; t < Steps; t++)
                {
                    float noiseSteer = 0f;

                    if (s0.v < 3.0f)
                    {
                        if (k < 10) noiseSteer = -1.0f;
                        else if (k < 20) noiseSteer = 1.0f;
                        else noiseSteer = UnityEngine.Random.Range(-1.0f, 1.0f);
                    }
                    else
                    {
                        float noiseScale = 0.4f;
                        noiseSteer = (k == 0) ? 0 : UnityEngine.Random.Range(-noiseScale, noiseScale);
                    }

                    float uSteer = 0f;
                    if (s0.v < 4.0f && k < 30) uSteer = Mathf.Clamp(noiseSteer, -1f, 1f);
                    else uSteer = Mathf.Clamp(_nominalSteer[t] + noiseSteer, -1f, 1f);

                    float noiseAccel = (k == 0) ? 0 : UnityEngine.Random.Range(-0.3f, 0.3f);
                    float uAccel = Mathf.Clamp(_nominalAccel[t] + noiseAccel, -1f, 1f);

                    sampleSteers[k][t] = uSteer;
                    sampleAccels[k][t] = uAccel;

                    simState = BicycleModelStep(simState, uSteer, uAccel, _dt);
                    _debugTrajectories[k].Add(new Vector3(simState.x, s0.y, simState.z));

                    totalCost += CalculateStageCost(simState, uSteer, uAccel, prevSteer, s0.v);
                    prevSteer = uSteer;

                    if (totalCost > 5000f) break;
                }

                sampleCosts[k] = totalCost;
                if (totalCost < minCost) minCost = totalCost;
            }

            float lambda = 3.0f;
            float totalWeight = 0f;
            float[] weights = new float[NumSamples];

            for (int k = 0; k < NumSamples; k++)
            {
                float w = Mathf.Exp(-(sampleCosts[k] - minCost) / lambda);
                weights[k] = w;
                totalWeight += w;
            }

            if (totalWeight > 1e-5f)
            {
                for (int t = 0; t < Steps; t++)
                {
                    float weightedSteer = 0f;
                    float weightedAccel = 0f;

                    for (int k = 0; k < NumSamples; k++)
                    {
                        float normW = weights[k] / totalWeight;
                        weightedSteer += sampleSteers[k][t] * normW;
                        weightedAccel += sampleAccels[k][t] * normW;
                    }
                    _nominalSteer[t] = Mathf.Lerp(_nominalSteer[t], weightedSteer, 0.8f);
                    _nominalAccel[t] = Mathf.Lerp(_nominalAccel[t], weightedAccel, 0.8f);
                }
            }

            State bestSim = s0;
            for (int t = 0; t < Steps; t++)
            {
                bestSim = BicycleModelStep(bestSim, _nominalSteer[t], _nominalAccel[t], _dt);
                _bestTrajectoryPoints[t] = new Vector3(bestSim.x, s0.y, bestSim.z);
            }
        }

        private State BicycleModelStep(State s, float steerNorm, float accelNorm, float dt)
        {
            float currentMaxSteer = GetDynamicMaxSteer(s.v);
            float delta = (-steerNorm) * currentMaxSteer * Mathf.Deg2Rad;

            float a = accelNorm * ((accelNorm > 0) ? 5.0f : 8.0f);

            float x_new = s.x + s.v * Mathf.Cos(s.yaw) * dt;
            float z_new = s.z + s.v * Mathf.Sin(s.yaw) * dt;
            float yaw_new = s.yaw + (s.v / WheelBase) * Mathf.Tan(delta) * dt;
            float v_new = Mathf.Clamp(s.v + a * dt, -2f, 25f);

            return new State { x = x_new, z = z_new, yaw = yaw_new, v = v_new, y = s.y };
        }

        private float CalculateStageCost(State s, float steer, float accel, float prevSteer, float currentRealSpeed)
        {
            float cost = 0;

            float rightYaw = s.yaw - (Mathf.PI / 2.0f);
            float checkDist = VehicleWidth / 2.0f;
            float dx = Mathf.Cos(rightYaw) * checkDist;
            float dz = Mathf.Sin(rightYaw) * checkDist;

            Vector3[] checkPoints = new Vector3[] {
                new Vector3(s.x, s.y, s.z),
                new Vector3(s.x + dx, s.y, s.z + dz),
                new Vector3(s.x - dx, s.y, s.z - dz)
            };

            if (_obstacleMap != null)
            {
                bool hit = false;
                foreach (var p in checkPoints)
                {
                    var trav = _obstacleMap.GetGlobalPointTravesibility(p);
                    if (trav == ObstacleMap.Traversability.Blocked)
                    {
                        hit = true;
                        break;
                    }
                }
                if (hit) return 10000.0f;
            }

            Vector3 carPos = new Vector3(s.x, s.y, s.z);
            int lookAhead = (s.v < 4.0f) ? 2 : 10;
            Vector3 target = Path.PeekWayPoint(lookAhead).WorldPos();

            if (target == Vector3.zero) target = Path.GetCurrentWayPoint().WorldPos();
            _currentDebugTarget = target;

            float distSq = (carPos - target).sqrMagnitude;
            cost += distSq * WeightDist;

            float speedDiff = s.v - TargetSpeed;
            cost += (speedDiff * speedDiff) * WeightSpeed;

            Vector3 diff = target - carPos;
            if (diff.sqrMagnitude > 0.1f)
            {
                float targetYaw = Mathf.Atan2(diff.z, diff.x);
                float yawErr = Mathf.Abs(Mathf.DeltaAngle(s.yaw * Mathf.Rad2Deg, targetYaw * Mathf.Rad2Deg));
                cost += yawErr * WeightHeading;
            }

            float dynamicSteerWeight = (currentRealSpeed < 3.0f) ? WeightSteerChange * 0.1f : WeightSteerChange;
            float steerChange = steer - prevSteer;
            cost += (steerChange * steerChange) * dynamicSteerWeight;

            float absSteerWeight = (currentRealSpeed < 3.0f) ? 0.01f : 0.1f;
            cost += (steer * steer) * absSteerWeight;

            return cost;
        }

        private State GetVehicleState()
        {
            Vector3 pos = Controller.transform.position;
            Rigidbody rb = Controller.GetComponent<Rigidbody>();
            float yaw = Mathf.Atan2(Controller.transform.forward.z, Controller.transform.forward.x);
            float v = (rb != null) ? Vector3.Dot(Controller.transform.forward, rb.linearVelocity) : 0f;
            return new State { x = pos.x, y = pos.y, z = pos.z, yaw = yaw, v = v };
        }

        private struct State { public float x, y, z, yaw, v; }

    }
}