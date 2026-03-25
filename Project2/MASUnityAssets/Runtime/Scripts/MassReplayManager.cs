using UnityEngine;
using UnityEngine.UI;
using System.IO;
using System.Text.RegularExpressions;
using Scripts.Map;

namespace Scripts
{
    public class MassReplayManager : MonoBehaviour
    {
        public MapManager m_MapManager;
        public GameObject replayVehiclePrefab;

        public string massReplayKeyword = "car";

        private void Start()
        {
            SetupReplay();
        }

        void SetupReplay()
        {
            string folderName = Application.dataPath + "/StreamingAssets/Text/";
            if (Directory.Exists(folderName))
            {
                GameObject spawn_vehicle;
                TrajectoryLogger spawn_logger;
                Text spawn_text;
                string group_number;

                DirectoryInfo d = new DirectoryInfo(folderName);
                foreach (var file in d.GetFiles("*.json"))
                {
                    if (file.Name.ToLower().Contains(massReplayKeyword.ToLower()))
                    {
                        Debug.Log(file.Name);
                        spawn_vehicle = Instantiate(replayVehiclePrefab, m_MapManager.GetGlobalStartPosition(), Quaternion.identity, m_MapManager.transform);
                        spawn_logger = spawn_vehicle.GetComponent<TrajectoryLogger>();
                        spawn_logger.trajectory_filename = file.Name.Replace(".json", "");
                        spawn_logger.SetJsonFile();
                        spawn_logger.PlaybackOn = true;
                        spawn_text = spawn_vehicle.GetComponentInChildren<Text>();
                        group_number = Regex.Match(file.Name, @"-?\d+").Value;
                        spawn_text.text = group_number;
                    }
                }
            }
        }
    }
}