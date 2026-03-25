using System;
using System.Collections.Generic;
using System.Linq;
using Path_Planning;
using UnityEngine;

public class Path
{
    private List<State> GlobalPath;
    private int CurrentIdx = 0;

    public Path()
    {
        this.GlobalPath = new List<State>();
    }
    public Path(List<State> wayPoints)
    {
        this.GlobalPath = new List<State>(wayPoints);
    }

    public bool IsInitialized()
    {
        return this.GlobalPath.Count > 0;
    }

    public State GetCurrentWayPoint()
    {
        return this.GlobalPath[CurrentIdx];
    }

    public int GetCurrentIdx()
    {
        return this.CurrentIdx;
    }

    public float GetDistanceToPath(State other)
    {
        return GetSubPath(CurrentIdx, GlobalPath.Count - 1).GetWayPoints().Min(node => node.DistanceToPos(other));
    }

    public State GetClosest(State other)
    {
        return GlobalPath.OrderBy(node => node.DistanceToPos(other)).First();
    }

    public void JumpTo(State other)
    {
        var idx = GlobalPath.IndexOf(GetClosest(other));
        if (idx < CurrentIdx) return;
        CurrentIdx = Math.Min(idx, this.GlobalPath.Count - 1);
    }

    public int GetLength()
    {
        return this.GlobalPath.Count;
    }

    public int GetWayPointsLeftCount()
    {
        return this.GlobalPath.Count - this.CurrentIdx - 1;
    }

    public bool ReachedEnd()
    {
        return this.CurrentIdx == this.GlobalPath.Count - 1;
    }

    public bool Contains(State waypoint)
    {
        return this.GlobalPath.Contains(waypoint);
    }
    
    public bool Contains(Vector3 waypointPosition)
    {
        return this.GlobalPath.Any(wp => Vector3.Distance(waypointPosition, wp.WorldPos()) < 0.1f);
    }

    public void Step()
    {
        if (!this.ReachedEnd()) this.CurrentIdx++;
    }

    public Path GetSubPath(int startIdx, int endIdx)
    {
        if (startIdx > GetLength() - 1 || endIdx < startIdx) return new Path();
        if (endIdx > GetLength() - 1) endIdx = GetLength() - 1;
        return new Path(GlobalPath.GetRange(startIdx, endIdx - startIdx));
    }

    public void ReplaceSubPath(Path subPath, int startIdx, int endIdx)
    {
        if (startIdx > GetLength() - 1) return;
        var newPath = new List<State>();
        if (startIdx > 0) newPath.AddRange(GlobalPath.GetRange(0,startIdx));
        newPath.AddRange(subPath.GetWayPoints());
        newPath.AddRange(GetSubPath(endIdx, GetLength() - 1).GetWayPoints());
        GlobalPath = newPath;
    }

    public State GetWayPoint(int idx)
    {
        if (GlobalPath.Count == 0) return new State(0,0,0,0);
        idx = Mathf.Clamp(idx, 0, GlobalPath.Count - 1);
        return GlobalPath[idx];
    }

    public State PeekWayPoint(int offset)
    {
        return GetWayPoint(CurrentIdx + offset);
    }

    public IReadOnlyList<State> GetWayPoints()
    {
        return GlobalPath;
    }

    public void StepUntilFar(Vector3 currentPos, float threshold)
    {
        while (!ReachedEnd() && Vector3.Distance(GetCurrentWayPoint().WorldPos(), currentPos) < threshold)
        {
            CurrentIdx++;
        }
    }
    
    public void DebugDraw()
    {
        this.DebugDraw(Color.aquamarine, Color.red, Color.orange);
    }

    public void DebugDraw(Color colorBefore, Color colorCurrent, Color colorAfter)
    {
        if (GlobalPath.Count > 0)
        {
            var reachedCurrent = false;
            for (var i = 0; i < GlobalPath.Count /*- 1*/; i++)
            {
                if (GlobalPath[i] == GetCurrentWayPoint())
                {
                    Gizmos.color = colorCurrent;
                    reachedCurrent = true;
                }
                else if (reachedCurrent)
                {
                    Gizmos.color = colorAfter;
                }
                else
                {
                    Gizmos.color = colorBefore;
                }
                //Gizmos.DrawLine(GlobalPath[i], GlobalPath[i+1]);
                Gizmos.DrawSphere(GlobalPath[i].WorldPos(), 0.5f);
            }
        }
    }
}