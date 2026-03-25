using System;
using System.Collections.Generic;
using System.Linq;
using Scripts.Utils;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Scripts.Map
{
    public class MapManager : MonoBehaviour
    {
        public List<Vector3> startPositions;
        public List<Vector3> targetPositions;

        public String fileName;

        private void Awake()
        {
            Initialize();
        }

        public List<GameObject> GetObstacleObjects()
        {
            return transform.FindAllChildrenWithTag("Obstacle");
        }

        public Vector3 GetGlobalStartPosition()
        {
            return transform.TransformPoint(startPositions[0]);
        }

        public Vector3 GetGlobalGoalPosition()
        {
            return transform.TransformPoint(targetPositions[0]);
        }

        public void Initialize()
        {
            startPositions = transform.FindAllChildrenWithTag("Start").ToList().Select(transf => transform.InverseTransformPoint(transf.transform.position)).ToList();
            targetPositions = transform.FindAllChildrenWithTag("Target").ToList().Select(transf => transform.InverseTransformPoint(transf.transform.position)).ToList();
        }

        public void ClearMap()
        {
            List<GameObject> children = FetchChildren(gameObject);
            foreach (var child in children)
            {
                DestroyImmediate(child);
            }
        }

        public void SaveMap()
        {
            var saveInfo = SaveDataV3.PrepareSaveData(this);
            var json = JsonUtility.ToJson(saveInfo);
            FileUtils.WriteJsonToFile(json, "/Maps/" + fileName);
        }

        public void LoadMap(string option)
        {
            var readJsonFromFile = FileUtils.ReadMapFromFile(option);

            var saveData = JsonUtility.FromJson<SaveDataV3>(readJsonFromFile);
            SaveDataV3.LoadSaveDataV3(saveData, this);

            Initialize();
            fileName = option;
        }

        public static GameObject DoInstantiateObject(Object prefabObject)
        {
            return (GameObject)Instantiate(prefabObject, Vector3.zero, Quaternion.identity);
        }


        private List<GameObject> FetchChildren(GameObject gameObject)
        {
            List<GameObject> children = new List<GameObject>();
            foreach (Transform child in gameObject.transform)
            {
                children.Add(child.gameObject);
            }

            return children;
        }

        public List<GameObject> GetStartObjects()
        {
            return transform.FindAllChildrenWithTag("Start").ToList();
        }

        public List<GameObject> GetTargetObjects()
        {
            return transform.FindAllChildrenWithTag("Target").ToList();
        }
    }
}