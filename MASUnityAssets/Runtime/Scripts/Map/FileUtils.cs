using System;
using System.Collections.Generic;
using System.IO;
//using Newtonsoft.Json;
using UnityEngine;

namespace Scripts.Map
{
    public static class FileUtils
    {
        private static readonly Dictionary<String, Material> MaterialCache = new();
        private static readonly Dictionary<String, UnityEngine.Object> ObjectCache = new();

        public static Material LoadMaterialFromFile(string filename)
        {
            if (MaterialCache.ContainsKey(filename)) return MaterialCache[filename];

            var material = Resources.Load(filename, typeof(Material)) as Material;
            MaterialCache[filename] = material;
            return material;
        }

        public static UnityEngine.Object LoadPrefabFromFile(string filename, bool suppressWarning = false)
        {
            filename = filename.Split("(")[0].Trim();
            // DirectoryInfo info = new DirectoryInfo("Prefabs/Kenneys");
            // info.GetDirectories();
            // TODO: Write folder iterator

            // if (ObjectCache.ContainsKey(filename)) return ObjectCache[filename];

            var loadedObject = Resources.Load(filename);

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/RuleTileAssets/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/Platformer/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/CityKitCommercial/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/CityKitSuburban/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/CityKitRoads/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/Nature/" + filename);
            }

            if (loadedObject == null && !suppressWarning)
            {
                Debug.LogWarning("Prefab file not found (" + filename + ")... - please check the configuration");
            }

            // ObjectCache[filename] = loadedObject;
            return loadedObject;
        }

        public static string ReadMapFromFile(string filename)
        {
            return ReadJsonFromFile("Maps/" + filename);
        }

        public static string ReadJsonFromFile(string filename)
        {
            string textContent = "";

            if (filename != null && filename.Length > 0)
            {
                string path = Application.dataPath + "/Resources/Text/" + filename + ".json";
                try
                {
                    textContent = File.ReadAllText(path);
                }
                catch (Exception)
                {
                }

                if (textContent.Length == 0)
                {
                    path = Application.streamingAssetsPath + "/Text/" + filename + ".json";
                    textContent = File.ReadAllText(path);
                }
            }


            if (textContent.Length == 0)
            {
                throw new FileNotFoundException("No file found trying to load text from file (" + filename + ")... - please check the configuration");
            }

            return textContent;
        }

        public static void WriteMapToFile(string jsonString, string filename)
        {
            WriteJsonToFile(jsonString, "Maps/" + filename);
        }

        public static void WriteJsonToFile(string jsonString, string filename)
        {
            if (filename != null && filename.Length > 0)
            {
                string path = Application.dataPath + "/StreamingAssets/Text/" + filename + ".json";
                Debug.Log("Writing to AssetPath:" + path);
                File.WriteAllText(path, jsonString);
#if UNITY_EDITOR
                UnityEditor.AssetDatabase.Refresh();
#endif
            }
        }
    }
}