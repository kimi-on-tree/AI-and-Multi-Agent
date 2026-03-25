using Path_Execution;
using Path_Planning;
using Scripts.Game;
using Scripts.Vehicle;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : Agent
{
    public bool drawMapGizmo;
    public int verbosity = 0;
    public DroneController m_Drone;

    private PathPlanner _pathPlanner;
    private PathExecutor _pathExecutor;

    public override void Initialize()
    {
        var specificPlanner = new MovementTreePlanner(MapManager, transform.position, MapManager.GetGlobalGoalPosition(),
             new DroneDynamics());
        //set up path planner and plan path
        _pathPlanner = new MovementTreePlanner(MapManager, transform.position, MapManager.GetGlobalGoalPosition(),
             new CarDynamics());
        _pathPlanner.PlanPath();
        Path rawPath = _pathPlanner.GetPath();
        var smoother = new MovementPathSmoother(specificPlanner.GetObstacleRiskGrid(), new DroneDynamics());
        smoother.SmoothPath(rawPath);
        //set up path executor
        //TODO: make executor take any controller, not just car
        //_pathExecutor = new PDPathFollower(_pathPlanner.GetPath(), m_Drone);
        _pathExecutor = new SimpleDronePathFollower(_pathPlanner.GetPath(), m_Drone);
    }

    public override void Step()
    {
        // this is how you control the drone. Example with an arbitrary sine function.
        //m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);
        if (_pathExecutor != null)
        {
            _pathExecutor.Step();
        }
    }
    
    void OnDrawGizmos()
    {
        if (drawMapGizmo)
        {
            _pathPlanner?.DebugDraw(verbosity);
            _pathPlanner?.GetPath().DebugDraw();
            _pathExecutor?.DebugDraw();
        }

    }

   
}