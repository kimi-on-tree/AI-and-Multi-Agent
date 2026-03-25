using System;
using System.Collections.Generic;
using UnityEngine;

namespace Scripts.Utils
{
    public static class TransformUtils
    {
        public static List<Transform> GetChildren(this Transform gameObject)
        {
            List<Transform> returnList = new();
            foreach (Transform childTransform in gameObject.transform)
            {
                returnList.Add(childTransform);
            }

            return returnList;
        }
        
        public static List<GameObject> FindAllChildrenWithTag(this Transform item, String tag)
        {
            var objects = new List<GameObject>();
            foreach (Transform child in item)
            {
                if (child.CompareTag(tag))
                {
                    objects.Add(child.gameObject);
                }

                objects.AddRange(child.FindAllChildrenWithTag(tag));
            }

            return objects;
        }
    }
}