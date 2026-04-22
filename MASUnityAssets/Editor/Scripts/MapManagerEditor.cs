using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace Scripts.Map
{
    [CustomEditor(typeof(MapManager))]
    public class MapManagerEditor : UnityEditor.Editor
    {
        private int index = 0;
        private string[] options = { };

        private void OnEnable()
        {
            MapManager myScript = (MapManager)target;
            myScript.Initialize();
        }


        public override void OnInspectorGUI()
        {
            var changed = DrawDefaultInspector();
            MapManager myScript = (MapManager)target;

            if (changed)
            {
                myScript.Initialize();
            }

            if (GUILayout.Button("Save") && myScript.fileName.Length > 0)
            {
                myScript.SaveMap();
                options = findFiles();
            }
            
            GUILayout.Space(15);
            
            if (options.Length == 0) options = findFiles();
            index = EditorGUILayout.Popup(index, options);

            if (GUILayout.Button("Load") && options.Length > 0)
            {
                myScript.ClearMap();
                myScript.LoadMap(options[index]);
            }

            if (GUILayout.Button("Refresh maps"))
            {
                options = findFiles();
            }
            
            if (GUILayout.Button("Clear map"))
            {
                myScript.ClearMap();
            }
            
            GUILayout.Space(15);
        }

        public string[] findFiles()
        {
            var info = new DirectoryInfo(Application.dataPath + "/StreamingAssets/Text/Maps/");
            var fileInfo = info.GetFiles();
            return fileInfo.ToList().FindAll(file => file.Name.EndsWith(".json")).Select(file => file.Name.Replace(".json", "")).ToArray();
        }
    }
}