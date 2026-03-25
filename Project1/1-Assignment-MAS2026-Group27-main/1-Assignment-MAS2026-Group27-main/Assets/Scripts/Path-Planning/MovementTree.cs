using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Path_Planning
{
    public class MovementTree
    {
        private Dictionary<State, State> _childParent = new();
        private Dictionary<State, MovementPrimitive> _childMovement = new();
        private State _root;
        private static Color _colorPath = Color.greenYellow;
        private static Color _colorHeading = Color.cornflowerBlue;
    
        public MovementTree(State root)
        {
            _root = root;
        }

        public State GetRoot()
        {
            return _root.Copy();
        }

        public void Set(State parent, State child, MovementPrimitive movement)
        {
            if (_childParent.Keys.Count == 0 && parent != _root) return;
            _childParent[child] = parent;
            _childMovement[child] = movement;
        }

        public bool HasChildren(State node)
        {
            return _childParent.ContainsValue(node);
        }

        public bool HasParent(State node)
        {
            return _childParent.ContainsKey(node);
        }

        public State GetParent(State child)
        {
            return _childParent[child];
        }

        public List<State> GetChildren(State parent)
        {
            return _childParent.Keys.Where(child => _childParent[child] == parent).ToList();
        }

        public MovementPrimitive GetMovementTo(State child)
        {
            return _childMovement[child];
        }

        public List<State> GetNodes()
        { 
            var nodes = new List<State>(_childParent.Keys) { _root };
            return nodes;
        }

        public State GetClosestNodeTo(Vector3 pos)
        {
            return GetNodes().OrderBy(n => n.DistanceToPos(pos)).First();
        }
    
        public void DebugDraw()
        {
            var openSet = new List<State>();
            openSet.Add(_root);
            while (openSet.Count > 0)
            {
                var newOpenSet = new List<State>();
                foreach (var node in openSet.Where(node => HasChildren(node)))
                {
                    var children = GetChildren(node);
                    foreach (var child in children)
                    {
                        //draw path piece
                        var childVector = new Vector3(child.X, 0.2f, child.Y);
                        Gizmos.color = _colorPath;
                        Gizmos.DrawLine(new Vector3(node.X, 0.2f, node.Y), childVector);
                    
                        //draw heading
                        var heading = new Vector3((float)Math.Cos(child.Heading), 0f, (float)Math.Sin(child.Heading));
                        Gizmos.color = _colorHeading;
                        Gizmos.DrawLine(childVector, childVector + heading);
                    
                        newOpenSet.Add(child);
                    }
                }
                openSet = newOpenSet;
            }
        }



    }
}