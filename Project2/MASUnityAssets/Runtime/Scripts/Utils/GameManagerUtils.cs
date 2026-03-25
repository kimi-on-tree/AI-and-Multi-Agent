using System.Collections.Generic;
using System.Linq;
using Scripts.Map;
using UnityEngine;

namespace Scripts.Utils
{
    public static class GameManagerUtils
    {
        public static List<GameObject> CreatePrefabs(List<Transform> startPositions, MapManager parent, GameObject prefab)
        {
            var vehicleAssignments = new List<GameObject>();
            foreach (var startPosition in startPositions)
            {
                var vehicleInstance = Object.Instantiate(prefab,
                    startPosition.position + prefab.transform.localPosition,
                    startPosition.rotation,
                    parent.transform
                );

                vehicleAssignments.Add(vehicleInstance);
            }

            return vehicleAssignments;
        }

        public static T CopyComponent<T>(T original, GameObject destination) where T : Component
        {
            System.Type type = original.GetType();
            Component copy = destination.AddComponent(type);
            System.Reflection.FieldInfo[] fields = type.GetFields();
            foreach (System.Reflection.FieldInfo field in fields)
            {
                field.SetValue(copy, field.GetValue(original));
            }

            return copy as T;
        }
    }
}