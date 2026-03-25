using Scripts.Map;
using UnityEditor;

namespace Editor.Scripts
{
    [CustomEditor(typeof(ObstacleMapVisualizer))]
    public class ObstacleMapVisualizerEditor : UnityEditor.Editor
    {
        private void OnEnable()
        {
            ObstacleMapVisualizer myScript = (ObstacleMapVisualizer)target;
            myScript.Start();
        }


        public override void OnInspectorGUI()
        {
            var changed = DrawDefaultInspector();

            ObstacleMapVisualizer myScript = (ObstacleMapVisualizer)target;
            if (changed && myScript.drawMapGizmo)
            {
                myScript.Start();
            }
        }
    }
}