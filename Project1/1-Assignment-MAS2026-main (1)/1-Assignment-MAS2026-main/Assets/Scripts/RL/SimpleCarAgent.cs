using Imported.StandardAssets.Vehicles.Car.Scripts;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace ML
{
    public class SimpleCarAgent : Agent
    {
        public GameObject target;

        private CarController carController;
        private Rigidbody rigidBody;

        public override void Initialize()
        {
            carController = GetComponent<CarController>();
            rigidBody = GetComponent<Rigidbody>();
        }

        public override void OnEpisodeBegin()
        {
            target.transform.localPosition = new Vector3(10f + Random.value * 110f, 1f, -10f + Random.value * 100f);

            do transform.localPosition = new Vector3(10f + Random.value * 110f, 0.1f, -10f + Random.value * 100f);
            while ((transform.localPosition - target.transform.localPosition).magnitude < 30);
            transform.localRotation = Quaternion.Euler(0, Random.value * 360, 0);

            rigidBody.linearVelocity = Vector3.zero;
            rigidBody.angularVelocity = Vector3.zero;
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            // Normally continuous actions are in the range of -1 to 1.
            // Depending on various parameters used in training, this might not be the case.
            var steering = Mathf.Clamp(actions.ContinuousActions[0], -1, 1);
            var v = Mathf.Clamp(actions.ContinuousActions[1], -1, 1);

            carController.Move(steering, v, v, 0);

            if (MaxStep != 0) //With infinite episodes, need to handle reward a different way
            {
                // An example dense reward based on distance. 0 when further than 50 units away from target.
                var distanceReward = 1 - Mathf.Clamp01((transform.localPosition - target.transform.localPosition).magnitude / 50);
                AddReward(distanceReward / MaxStep);
                // Debug.Log(GetCumulativeReward());
            }
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            sensor.AddObservation(target.transform.localRotation.eulerAngles / 360f);
            sensor.AddObservation(Vector3.ClampMagnitude(transform.InverseTransformVector(rigidBody.linearVelocity) / 50, 1));
            sensor.AddObservation(Vector3.ClampMagnitude(transform.InverseTransformVector(rigidBody.angularVelocity) / 5, 1));
            sensor.AddObservation(Vector3.ClampMagnitude(transform.InverseTransformVector((target.transform.position - transform.position)) / 150, 1));
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var actionsOutContinuousActions = actionsOut.ContinuousActions;
            actionsOutContinuousActions[0] = Input.GetAxis("Horizontal");
            actionsOutContinuousActions[1] = Input.GetAxis("Vertical");
        }
    }
}