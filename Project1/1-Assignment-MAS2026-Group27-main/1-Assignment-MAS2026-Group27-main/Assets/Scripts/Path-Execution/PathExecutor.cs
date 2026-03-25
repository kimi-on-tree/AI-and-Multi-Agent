using Imported.StandardAssets.Vehicles.Car.Scripts;
using UnityEngine;

namespace Path_Execution
{
    public abstract class PathExecutor
    {
        protected Path Path;
        protected MonoBehaviour Controller;
    
        protected PathExecutor(Path path, MonoBehaviour controller)
        {
            this.Path = path;
            this.Controller = controller;
        }
    
        public abstract void Step();

        public abstract void DebugDraw();
    }
}
