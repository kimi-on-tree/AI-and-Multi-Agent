using System;
using System.Collections.Generic;
using UnityEngine;

namespace Path_Planning
{
    public class DroneDynamics: VehicleDynamics
    {
        private readonly float _maxVelocity = 15f;
        private float _maxAccel = 15f;

        public override float MaxVelocity => 5f;
        public override float MaxDeceleration => 5f;   
        public override float TurnSpeedLimitRef => 45f; 
        public override float MinTurnSpeed => 3f;

        private static float DroneMaximumSteerAngle = Mathf.PI;

        private static List<MovementPrimitive> movementPrimitives = new()
        {
            //full accel
            new MovementPrimitive(accel: 1, steering: 0),
            new MovementPrimitive(accel: 1, steering: 0.25f),
            new MovementPrimitive(accel: 1, steering: 0.5f),
            new MovementPrimitive(accel: 1, steering: 0.75f),
            new MovementPrimitive(accel: 1, steering: 1),
            new MovementPrimitive(accel: 1, steering: -0.25f),
            new MovementPrimitive(accel: 1, steering: -0.5f),
            new MovementPrimitive(accel: 1, steering: -0.75f),
            //half accel
            new MovementPrimitive(accel: 0.5f, steering: 0),
            new MovementPrimitive(accel: 0.5f, steering: 0.25f),
            new MovementPrimitive(accel: 0.5f, steering: 0.5f),
            new MovementPrimitive(accel: 0.5f, steering: 0.75f),
            new MovementPrimitive(accel: 0.5f, steering: 1),
            new MovementPrimitive(accel: 0.5f, steering: -0.25f),
            new MovementPrimitive(accel: 0.5f, steering: -0.5f),
            new MovementPrimitive(accel: 0.5f, steering: -0.75f)
        };

        public DroneDynamics() : base(movementPrimitives)
        {
            VehicleRadius = 3.0f * 1.0f;
            PlanningNodeDistanceThreshold = 0.2f;
            SimulationDistance = 2f;
            SimulationStepCount = 10;
        }

        protected override State MakeStep(State state, MovementPrimitive movement)
        {
            var steeringAngle = movement.Steering * DroneMaximumSteerAngle;
            var deltaT = DeltaT;
            if (Math.Abs(state.Velocity) >= 0.5) deltaT /= Math.Abs(state.Velocity);
            
            var newVelocity = AngleVector(state.Heading) * state.Velocity;
            newVelocity += movement.Accel * AngleVector(steeringAngle) * deltaT;
            if (newVelocity.magnitude >= _maxVelocity) newVelocity *= (_maxVelocity / newVelocity.magnitude);

            var newHeading = state.Heading;
            if (newVelocity.magnitude > 0.1f)
            {
                newHeading = Mathf.Atan2(newVelocity.x, newVelocity.y) * Mathf.Rad2Deg;
            }

            var newX = state.X + newVelocity.x * deltaT;
            var newY = state.Y + newVelocity.y * deltaT;

            return new State(newX, newY, newHeading, newVelocity.magnitude);
        }

        public override void UpdateMaxAccel(float accel)
        {
            _maxAccel = accel;
        }

        private static Vector2 AngleVector(float angle)
        {
            return new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));
        }
    }
}