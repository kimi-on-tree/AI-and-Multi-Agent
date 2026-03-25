using System;
using System.Collections.Generic;

namespace Path_Planning
{
    public class CarDynamics: VehicleDynamics
    {
        public static readonly float WheelBase = 2.4f;
        //public static readonly float MaxVelocity = 5f;
        public static readonly float MaxSteerAngle = 26f / 180f * (float)Math.PI;

        public override float MaxVelocity => 15f;
        public override float MaxDeceleration => 3f;    
        public override float TurnSpeedLimitRef => 30f; 
        public override float MinTurnSpeed => 1f;

        private static List<MovementPrimitive> movementPrimitives = new()
        {
            //full forward
            new MovementPrimitive(accel: 1, steering: 0),
            new MovementPrimitive(accel: 1, steering: 1),
            new MovementPrimitive(accel: 1, steering: -1),
            new MovementPrimitive(accel: 1, steering: 0.5f),
            new MovementPrimitive(accel: 1, steering: -0.5f),
            //half forward
            new MovementPrimitive(accel: 0.5f, steering: 0),
            new MovementPrimitive(accel: 0.5f, steering: 1),
            new MovementPrimitive(accel: 0.5f, steering: -1),
            new MovementPrimitive(accel: 0.5f, steering: 0.5f),
            new MovementPrimitive(accel: 0.5f, steering: -0.5f),
            //no acceleration
            new MovementPrimitive(accel: 0, steering: 0),
            new MovementPrimitive(accel: 0, steering: 1),
            new MovementPrimitive(accel: 0, steering: -1),
            //full reverse
            new MovementPrimitive(accel: -1, steering: 0),
            new MovementPrimitive(accel: -1, steering: 1),
            new MovementPrimitive(accel: -1, steering: -1),
            new MovementPrimitive(accel: -1, steering: 0.5f),
            new MovementPrimitive(accel: -1, steering: -0.5f),
            //half reverse
            new MovementPrimitive(accel: -0.5f, steering: 0),
            new MovementPrimitive(accel: -0.5f, steering: 1),
            new MovementPrimitive(accel: -0.5f, steering: -1),
            new MovementPrimitive(accel: -0.5f, steering: 0.5f),
            new MovementPrimitive(accel: -0.5f, steering: -0.5f)
        };
        public CarDynamics(): base(movementPrimitives)
        {
            VehicleRadius = 2.574f * 0.8f;
            SimulationDistance = 1f;
            SimulationStepCount = 15;
        }

        protected override State MakeStep(State state, MovementPrimitive movement)
        {
            var steeringAngle = movement.Steering * MaxSteerAngle;
            if (state.Velocity > 6) steeringAngle *= (1 - ((state.Velocity - 6) / (MaxVelocity * 1.5f)));
            var deltaT = DeltaT;
            if (Math.Abs(state.Velocity) >= 0.5) deltaT /= Math.Abs(state.Velocity);
            var newX = state.X + state.Velocity * Math.Cos(state.Heading) * deltaT;
            var newY = state.Y + state.Velocity * Math.Sin(state.Heading) * deltaT;
            var newHeading = state.Heading + state.Velocity / WheelBase * Math.Tan(steeringAngle) * deltaT;
            var newVelocity = state.Velocity + movement.Accel * deltaT;
            newVelocity = Math.Clamp(newVelocity, 0f, MaxVelocity);
            
            return new State((float)newX, (float)newY, (float)newHeading, (float)newVelocity);
        }

        public override void UpdateMaxAccel(float accel)
        {
        }
    }
}