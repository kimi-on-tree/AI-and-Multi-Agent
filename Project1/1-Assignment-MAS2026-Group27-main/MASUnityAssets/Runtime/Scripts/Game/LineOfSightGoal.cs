using UnityEngine;

namespace Scripts.Game
{
    public class LineOfSightGoal : AbstractGoal
    {
        
        private readonly float maxTargetDistance;

        public LineOfSightGoal(GameObject target, float maxDistance)
        {
            targetObject = target;
            maxTargetDistance = maxDistance;
        }

        public override bool CheckAchieved(GameObject objectToCheck)
        {
            var targetPosition = targetObject.transform.position;
            if ((objectToCheck.transform.position - targetPosition).magnitude < maxTargetDistance)
            {
                var objectPosition = objectToCheck.transform.position;

                var direction = targetPosition - objectPosition;
                direction.Scale(new(1, 0, 1));

                var obscured = Physics.Raycast(objectPosition + Vector3.up, direction, out RaycastHit hit, direction.magnitude);
                if (!obscured && !achieved)
                {
                    completionTime = Time.time - startTime;
                    achieved = true;
                }
            }

            return achieved;
        }
    }
}