using System;
using System.Collections.Generic;
using System.Linq;
using Scripts.Map;
using Scripts.Utils;
using UnityEngine;
using System.Diagnostics;

namespace Scripts.Game
{
    
    [Serializable]
    public struct Statistics
    {
        public float simulationTime;
        public float realTime;
        
        public float controllerTimeMs;
        public float controllerTimePerAgentMs;
        public float controllerTimePerStepPerAgentMs;
        
        public float planningTime;
        public float planningTimePerAgent;

        public float numberOfCollisions;
        public float timeInCollision;

        public float distanceTravelledLongest;
        public float distanceTravelledAverage;
        public float distanceTravelledTotal;
        public float topSpeed;
        public float averageSpeed;
    }
    public abstract class AbstractGameManager : MonoBehaviour
    {
        public Statistics statistics;
        
        private float startTime;

        public MapManager mapManager;
        public GameObject vehiclePrefab;
        public Camera driveCamera;

        protected List<GameObject> vehicleList = new();
        protected List<Goal> goals;

        public abstract List<Goal> CreateGoals(List<GameObject> vehicles);
        public abstract bool IsDone();

        private int nrOfSteps = 0;
        protected virtual void Start()
        {
            if (vehiclePrefab == null) throw new Exception("No vehicle defined in game manager!");

            vehicleList = GameManagerUtils.CreatePrefabs(mapManager.GetStartObjects().Select(obj => obj.transform).ToList(), mapManager, vehiclePrefab);
            goals = CreateGoals(vehicleList);
            
            if (driveCamera != null)
            {
                AssignDriveCamera(driveCamera.GetComponent<FollowObject>(), vehicleList.First());
            }
        }


        private void AssignDriveCamera(FollowObject followObject, GameObject vehicleInstance)
        {
            if (followObject != null)
            {
                followObject.target_object = vehicleInstance.transform;
                followObject.CameraFixed = vehicleInstance.name.ToLower().Contains("drone");
                followObject.Start();
            }
        }
        
        private void FixedUpdate()
        {
            nrOfSteps++;
            var agents = FindObjectsByType<Agent>(FindObjectsSortMode.None);
            
            if (statistics.planningTime <= 0)
            {
                statistics.planningTime = agents.Sum(agent => agent.planningTimeMs) / 1000f;
                statistics.planningTimePerAgent = statistics.planningTime / agents.Length;
                startTime = Time.unscaledTime;
            }

            if (!IsDone())
            {
                statistics.simulationTime = Time.fixedTime;
                statistics.realTime = Time.unscaledTime - startTime;
                
                statistics.controllerTimeMs = agents.Sum(agent => agent.controllerTimeMs);
                statistics.controllerTimePerAgentMs = statistics.controllerTimeMs / agents.Length;
                statistics.controllerTimePerStepPerAgentMs = statistics.controllerTimeMs / nrOfSteps / agents.Length;

                statistics.numberOfCollisions = agents.Sum(agent => agent.numberOfCollisions);
                statistics.timeInCollision = agents.Sum(agent => agent.timeInCollision);

                statistics.distanceTravelledLongest = agents.Max(agent => agent.distanceTravelled);
                statistics.distanceTravelledTotal = agents.Sum(agent => agent.distanceTravelled);
                statistics.distanceTravelledAverage = agents.Sum(agent => agent.distanceTravelled) / agents.Length;
                statistics.topSpeed = agents.Max(agent => agent.topSpeed);
                statistics.averageSpeed = statistics.distanceTravelledAverage / statistics.simulationTime;
            }
        
        }
    }
}