using System.Collections.Generic;
using UnityEngine;

namespace Scripts.Map
{
    public class ObstacleMapVisualizer : MonoBehaviour
    {
        public MapManager mapManager;

        [Tooltip("Make sure to enable Gizmos in scene view!")]
        public bool drawMapGizmo;

        public Vector3 margin = Vector3.one; //Make public if you wish to use from editor.
        public Vector3 cellScale = Vector3.one;
        public ObstacleMap ObstacleMap => m_ObstacleMap;
        private ObstacleMap m_ObstacleMap;

        public void Start()
        {
            if (mapManager == null) mapManager = GetComponent<MapManager>();
            m_ObstacleMap = ObstacleMap.Initialize(mapManager,
                additionalObjects: new List<GameObject>(),
                cellScale: cellScale,
                margin: margin);
        }

    

        void OnDrawGizmos()
        {
            if (drawMapGizmo)
            {
                if (m_ObstacleMap == null)
                {
                    Start();
                }

                RenderObstacleMap();
            }
            else
            {
                m_ObstacleMap = null;
            }
        }

        private void RenderObstacleMap()
        {
            foreach (var posEntity in m_ObstacleMap.traversabilityPerCell)
            {
                var position = new Vector3Int(posEntity.Key.x, 0, posEntity.Key.y);

                var cellToWorld = m_ObstacleMap.CellToWorld(position) + m_ObstacleMap.trueScale / 2;
                cellToWorld.y = 0;
                cellToWorld += Vector3.up * 0.25f;

                var scale = Vector3.Scale(transform.localScale, m_ObstacleMap.trueScale);
                var gizmoSize = new Vector3(scale.x * 0.95f, 0.005f, scale.z * 0.95f);
                
                if (posEntity.Value == ObstacleMap.Traversability.Blocked)
                {
                    Gizmos.color = Color.red;
                }
                else
                {
                    Gizmos.color = Color.green;
                }

                Gizmos.DrawCube(cellToWorld, gizmoSize);
            }
        }
    }
}