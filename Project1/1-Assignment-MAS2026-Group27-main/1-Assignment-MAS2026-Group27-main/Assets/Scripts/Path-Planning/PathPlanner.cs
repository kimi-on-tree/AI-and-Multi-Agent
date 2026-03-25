using System.Collections.Generic;
using Scripts.Map;
using UnityEngine;

namespace Path_Planning
{
    public abstract class PathPlanner
    {
        protected MapManager MapManager;

        protected Vector3 StartPos;
        protected Vector3 GoalPos;

        protected Path Path;
        protected static readonly float ObstacleRiskDecayRate = 3.80f;
        
        protected VehicleDynamics _vehicleDynamics;

        protected PathPlanner(MapManager mapManager, Vector3 startPos, Vector3 goalPos, VehicleDynamics vehicleDynamics)
        {
            this.MapManager = mapManager;
            this.StartPos = startPos;
            this.GoalPos = goalPos;
            this._vehicleDynamics = vehicleDynamics;
            Path = new Path(new List<State>());
        }

        public abstract void PlanPath();

        public abstract Path GetPath();

        public abstract void DebugDraw(int verbosity);

    }
}