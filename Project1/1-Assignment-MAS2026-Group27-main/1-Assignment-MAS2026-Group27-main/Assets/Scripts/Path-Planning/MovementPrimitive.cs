namespace Path_Planning
{
    public class MovementPrimitive
    {
        public readonly float Accel;
        public readonly float ControllerAccel;
        public readonly float Steering;
        public readonly float FootBrake;

        public MovementPrimitive(float accel, float steering)
        {
            if (accel < 0)
            {
                ControllerAccel = -accel;
                FootBrake = -1;
            }
            else
            {
                ControllerAccel = accel;
                FootBrake = 1;
            }

            Accel = accel;
            Steering = steering % 360;
        }
    }
}