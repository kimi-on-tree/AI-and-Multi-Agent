using System;
using UnityEngine;

namespace Path_Planning
{
    public class State: IComparable<State>
    {
        public readonly float X;
        public readonly float Y;
        public readonly float Heading;
        public float Velocity;
        public float HeuristicValue = float.MaxValue;

        public State(float x, float y, float heading, float velocity)
        {
            X = x;
            Y = y;
            Heading = heading;
            Velocity = velocity;
        }

        public State Copy()
        {
            return new State(X, Y, Heading, Velocity);
        }

        public Vector3 WorldPos()
        {
            return new Vector3(X, 0, Y);
        }

        public float DistanceToPos(State other)
        {
            return Vector3.Distance(this.WorldPos(), other.WorldPos());
        }
        
        public float DistanceToPos(Vector3 pos)
        {
            return Vector3.Distance(this.WorldPos(), pos);
        }

        public float DistanceToHeading(State other)
        {
            return Math.Abs(this.Heading - other.Heading);
        }

        public int CompareTo(State other)
        {
            if (other == null) return 1;
            return HeuristicValue.CompareTo(other.HeuristicValue);
        }
    }
}