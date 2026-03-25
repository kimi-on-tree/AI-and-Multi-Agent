using Scripts.Game;
using Scripts.Vehicle;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : Agent
{
    public DroneController m_Drone;


    public override void Initialize()
    {
        // See the Car AI for more information
    }

    public override void Step()
    {
        // this is how you control the drone. Example with an arbitrary sine function.
        m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);
    }

   
}