using System.Collections.Generic;

namespace Path_Planning
{
    public abstract class VehicleDynamics
    {
        public float VehicleRadius;
        public float PlanningNodeDistanceThreshold = 0.04f;
        public readonly List<MovementPrimitive> MovementPrimitives;

        public virtual float MaxVelocity => 20f;      
        public virtual float MaxDeceleration => 3f;    
        public virtual float TurnSpeedLimitRef => 45f; 
        public virtual float MinTurnSpeed => 2f;

        protected static float SimulationDistance = 2f;
        protected static float SimulationStepCount = 10;
        protected static float DeltaT = SimulationDistance / SimulationStepCount;

        protected VehicleDynamics(List<MovementPrimitive> movementPrimitives)
        {
            MovementPrimitives = new List<MovementPrimitive>(movementPrimitives);
        }
        
        public State Move(State state, MovementPrimitive movement)
        {
            //simple bicycle model
            State newState = state;
            for (int i = 0; i < SimulationStepCount; i++)
            {
                newState = MakeStep(newState, movement);
            }

            return newState;
        }
        
        protected abstract State MakeStep(State state, MovementPrimitive movement);

        public abstract void UpdateMaxAccel(float accel);
    }
}