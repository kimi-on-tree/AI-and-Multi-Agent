using UnityEngine;

namespace Scripts.Game
{
    public interface Goal
    {
        public GameObject GetTargetObject();
        public bool CheckAchieved(GameObject objectToCheck);
        public bool IsAchieved();
        public void RestartTimer();
        public float CurrentTime();
    }
}