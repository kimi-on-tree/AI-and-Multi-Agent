using UnityEngine;

namespace Scripts.Game
{
    public class WaypointGoal : AbstractGoal
    {
        protected readonly Vector3 targetPosition;
        protected readonly float targetMaxDistance;


        public WaypointGoal(Vector3 position, float distanceMaxDistance)
        {
            targetPosition = position;
            targetMaxDistance = distanceMaxDistance;
        }

        public override bool CheckAchieved(GameObject objectToCheck)
        {
            if (achieved == false && (objectToCheck.transform.position - targetPosition).magnitude < targetMaxDistance)
            {
                completionTime = Time.time - startTime;
                achieved = true;
            }

            return achieved;
        }
    }
}