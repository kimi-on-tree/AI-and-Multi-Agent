using Imported.StandardAssets.CrossPlatformInput.Scripts;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;

namespace Scripts.Vehicle
{
    [RequireComponent(typeof(DroneController))]
    public class DroneUserControl : MonoBehaviour
    {
        private DroneController m_Drone;

        private void Awake()
        {
            m_Drone = GetComponent<DroneController>();
        }

        private void FixedUpdate()
        {
            float h = CrossPlatformInputManager.GetAxis("Horizontal");
            float v = CrossPlatformInputManager.GetAxis("Vertical");

            m_Drone.Move(h, v);
        }
    }
}