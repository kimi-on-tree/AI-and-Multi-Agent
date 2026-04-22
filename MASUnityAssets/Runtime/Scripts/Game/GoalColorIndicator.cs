using System;
using UnityEngine;

namespace Scripts.Game
{
    public class GoalColorIndicator : MonoBehaviour
    {
        public Color SuccessColor = Color.blue;
        private Goal goal;
        private ColorIndicator colorIndicator;

        public void Start()
        {
            colorIndicator = gameObject.GetComponent<ColorIndicator>();
        }
        public void SetByIndex(int key)
        {
            colorIndicator.SetByIndex(key);
        }

        public void SetGoal(Goal goal)
        {
            this.goal = goal;
        }

        private void FixedUpdate()
        {
            if (goal != null && goal.IsAchieved())
            {
                colorIndicator.SetColor(SuccessColor);
            }
        }
    }
}