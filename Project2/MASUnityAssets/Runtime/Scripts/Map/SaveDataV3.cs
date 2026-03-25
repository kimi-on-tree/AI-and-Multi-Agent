using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using Scripts.Utils;
using UnityEngine;
using UnityEngine.Tilemaps;

namespace Scripts.Map
{
    public class SaveDataV3
    {
        public List<SavedObject> SavedObjects;

        [Serializable]
        public struct SavedObject
        {
            public String name;
            public Vector3 position;
            public Quaternion rotation;
            public Vector3 scale;

            public bool isPrefab;
            public bool render;
            public bool hasCollider;
            public bool isConvex;
            public List<SavedObject> children;
            public string tag;
            public int layer;
        }


        public static void LoadSaveDataV3(SaveDataV3 saveData, MapManager mapObject)
        {
            foreach (var obj in saveData.SavedObjects)
            {
                InstantiateObject(obj, mapObject.gameObject);
            }
        }


        public static void InstantiateObject(SavedObject savedObject, GameObject parentObject)
        {
            var instantiated = savedObject.isPrefab ? MapManager.DoInstantiateObject(FileUtils.LoadPrefabFromFile(savedObject.name)) : new GameObject(savedObject.name);
            instantiated.transform.parent = parentObject.transform;

            instantiated.name = savedObject.name;
            instantiated.transform.localRotation = savedObject.rotation;
            instantiated.transform.localScale = savedObject.scale;
            instantiated.transform.localPosition = savedObject.position;
            instantiated.layer = savedObject.layer;
            instantiated.tag = savedObject.tag;

            if (savedObject.hasCollider && instantiated.GetComponent<Collider>() == null)
            {
                var collider = instantiated.AddComponent<MeshCollider>();
                collider.convex = savedObject.isConvex;
            }

            if (savedObject.children.Count > 0)
            {
                if (instantiated.transform.childCount == savedObject.children.Count)
                {
                    for (int i = 0; i < savedObject.children.Count; i++)
                    {
                        var child = instantiated.transform.GetChild(i);
                        child.localPosition = savedObject.children[i].position;
                        child.localRotation = savedObject.children[i].rotation;
                        child.localScale = savedObject.children[i].scale;
                        child.name = savedObject.children[i].name;
                    }
                }
                else
                {
                    foreach (var savedTileChild in savedObject.children)
                    {
                        InstantiateObject(savedTileChild, instantiated);
                    }
                }
            }
        }


        public static SaveDataV3 PrepareSaveData(MapManager mapManager)
        {
            var saveData = new SaveDataV3();

            saveData.SavedObjects = new List<SavedObject>();
            foreach (var child in mapManager.transform.GetChildren())
            {
                var saveObject = CreateSaveObjectData(child.gameObject);
                saveData.SavedObjects.Add(saveObject);
            }

            return saveData;
        }


        private static SavedObject CreateSaveObjectData(GameObject savedObject)
        {
            var meshCollider = savedObject.GetComponent<MeshCollider>();
            var children = new List<SavedObject>();
            var isPrefab = FileUtils.LoadPrefabFromFile(savedObject.name, true) != null;

            if (!isPrefab)
            {
                foreach (Transform child in savedObject.transform)
                {
                    children.Add(CreateSaveObjectData(child.gameObject));
                }
            }

            return new SavedObject()
            {
                name = savedObject.name,
                position = savedObject.transform.localPosition,
                rotation = savedObject.transform.localRotation,
                scale = savedObject.transform.localScale,
                tag = savedObject.tag,
                layer = savedObject.layer,
                isPrefab = isPrefab,
                hasCollider = meshCollider != null,
                isConvex = meshCollider != null && meshCollider.convex,
                children = children
            };
        }
    }
}