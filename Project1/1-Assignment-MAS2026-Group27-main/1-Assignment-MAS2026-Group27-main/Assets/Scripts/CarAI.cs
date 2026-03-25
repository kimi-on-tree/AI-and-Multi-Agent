using System.Collections.Generic;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Path_Execution;
using Path_Planning;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;

[RequireComponent(typeof(CarController))]
public class CarAI : Agent
{
    public bool drawMapGizmo;
    public int verbosity = 0;
    public CarController car; // the car controller we want to use
    private BoxCollider m_CarCollider;
    
    private MovementTreePlanner _pathPlanner;
    private PathExecutor _pathExecutor;

    public override void Initialize()  // Runs before game starts
    {
        m_CarCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();

        _pathPlanner = new MovementTreePlanner(MapManager, transform.position, MapManager.GetGlobalGoalPosition(),
            new CarDynamics());

        _pathPlanner.PlanPath();
        Path rawPath = _pathPlanner.GetPath();
        
        ObstacleRiskGrid riskGrid = _pathPlanner.GetObstacleRiskGrid();
        
        var smoother = new MovementPathSmoother(riskGrid, new CarDynamics());
        
        var time = Time.realtimeSinceStartup;
        smoother.SmoothPath(rawPath);
        Debug.Log("Finished path smoothing. Took " + (Time.realtimeSinceStartup - time) + " seconds.");
        
        _pathExecutor = new PDPathFollower(rawPath, car);
        
        
    }


    public override void Step() // Runs every step of the physics simulation
    {
        Vector3 globalPosition = transform.position;
        Debug.DrawLine(globalPosition, MapManager.GetGlobalStartPosition(), Color.cyan);
        Debug.DrawLine(globalPosition, MapManager.GetGlobalGoalPosition(), Color.blue);
        
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