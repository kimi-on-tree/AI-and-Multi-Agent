using UnityEngine;

namespace Scripts.Game
{
    public abstract class AbstractGoal : Goal
    {
        public abstract bool CheckAchieved(GameObject objectToCheck);

        protected bool achieved;
        protected float completionTime;
        protected float startTime;
        protected GameObject targetObject;



        public bool IsAchieved()
        {
            return achieved;
        }

        public void RestartTimer()
        {
            startTime = Time.fixedTime;
            //TODO Need to rewrite so this stuff is managed in a interface method
            achieved = false;
        }

        public float CurrentTime()
        {
            if (achieved) return completionTime;
            return Time.fixedTime - startTime;
        }

        public GameObject GetTargetObject()
        {
            return targetObject;
        }
    }
}