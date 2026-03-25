using System.Collections.Generic;
using UnityEngine;

namespace Path_Planning
{
    public class MovementPathSmoother
    {
        private ObstacleRiskGrid _obstacleRiskGrid;
        private VehicleDynamics _dynamics; // for different models

        // lookahead
        private const float LookAheadDistance = 3.0f;

        // VehicleDynamics 
        public MovementPathSmoother(ObstacleRiskGrid obstacleRiskGrid, VehicleDynamics dynamics)
        {
            _obstacleRiskGrid = obstacleRiskGrid;
            _dynamics = dynamics;
        }

        public Path SmoothPath(Path path)
        {
            if (path == null || !path.IsInitialized()) return path;

            // get waypoint list
            var points = path.GetWayPoints();
            int count = points.Count;

            // get restricted parameters for the model
            float maxVel = _dynamics.MaxVelocity;
            float maxDecel = _dynamics.MaxDeceleration;
            float turnRef = _dynamics.TurnSpeedLimitRef;
            float minTurnSpeed = _dynamics.MinTurnSpeed;

            // ========================================================
            // Forward Pass based on curvature
            // ========================================================
            for (int i = 0; i < count - 1; i++)
            {
                State current = points[i];
                float maxAngleDiff = 0f;

                // find max steer change
                for (int j = i + 1; j < count; j++)
                {
                    State future = points[j];

                    // calculate distance
                    float dist = Vector3.Distance(current.WorldPos(), future.WorldPos());
                    if (dist > LookAheadDistance) break;

                    // Calculate angle difference
                    float currentHeadingDeg = current.Heading * Mathf.Rad2Deg;
                    float futureHeadingDeg = future.Heading * Mathf.Rad2Deg;
                    float diff = Mathf.Abs(Mathf.DeltaAngle(currentHeadingDeg, futureHeadingDeg));

                    if (diff > maxAngleDiff) maxAngleDiff = diff;
                }

                // Calculate the speed limit factor
                // Curvature ratio: Maximum detected curvature / Reference curvature 
                float ratio = Mathf.Clamp01(maxAngleDiff / turnRef);

                // Use a square curve (ratio^2) for more aggressive deceleration:
                // Small bend (ratio=0.2) -> Factor=0.96 (almost no deceleration)
                // Medium bend (ratio=0.5) -> Factor=0.75
                // Large bend (ratio=0.8) -> Factor=0.36 (significant deceleration)
                float curveFactor = 1.0f - (ratio * ratio);

                // Lerp between minspeed and max
                float limitByCurvature = Mathf.Lerp(minTurnSpeed, maxVel, curveFactor);
                current.Velocity = Mathf.Min(current.Velocity, limitByCurvature);
            }

            // ========================================================
            // Backward Pass
            // v_current <= sqrt(v_next^2 + 2 * a * d)
            // ========================================================
            for (int i = count - 2; i >= 0; i--)
            {
                State current = points[i];
                State next = points[i + 1];

                float dist = Vector3.Distance(current.WorldPos(), next.WorldPos());

                float maxPhysicalVelocity = Mathf.Sqrt(Mathf.Pow(next.Velocity, 2) + 2 * maxDecel * dist);

                if (current.Velocity > maxPhysicalVelocity)
                {
                    current.Velocity = maxPhysicalVelocity;
                }
            }

            return path;
        }
    }
}