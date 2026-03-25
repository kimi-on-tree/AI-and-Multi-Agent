using System;
using System.Collections.Generic;
using Scripts.Map;
using UnityEngine;
using System.Diagnostics;

namespace Scripts.Game
{
    public abstract class Agent : MonoBehaviour
    {
        private Stopwatch _mControllerTimer;

        protected MapManager MapManager;
        protected ObstacleMap ObstacleMap;
        protected Rigidbody Rigidbody;

        public float planningTimeMs;
        public float controllerTimeMs;
        
        public float timeInCollision;
        public int numberOfCollisions;
        
        public float topSpeed;
        public float distanceTravelled;

        private void Start()
        {
            Rigidbody = GetComponent<Rigidbody>();
            MapManager = FindFirstObjectByType<AbstractGameManager>().mapManager;
            ObstacleMap = ObstacleMap.Initialize(MapManager, new List<GameObject>(), Vector3.one * 4);

            var planningTimer = Stopwatch.StartNew();
            Initialize();
            planningTimer.Stop();

            planningTimeMs = planningTimer.ElapsedMilliseconds;
            _mControllerTimer = new Stopwatch();
        }

        private void FixedUpdate()
        {
            _mControllerTimer.Start();
            Step();
            _mControllerTimer.Stop();
            controllerTimeMs = _mControllerTimer.ElapsedMilliseconds;

            topSpeed = Mathf.Max(topSpeed, Rigidbody.linearVelocity.magnitude);
            distanceTravelled += Rigidbody.linearVelocity.magnitude * Time.fixedDeltaTime;
        }

        private void OnCollisionEnter(Collision other)
        {
            if (other.gameObject.CompareTag("Obstacle") || 
                other.gameObject.CompareTag("Player")) numberOfCollisions++;
        }

        private void OnCollisionStay(Collision other)
        {
            if (other.gameObject.CompareTag("Obstacle") ||
                other.gameObject.CompareTag("Player")) timeInCollision += Time.fixedDeltaTime;
        }

        public abstract void Initialize();
        public abstract void Step();
    }
}