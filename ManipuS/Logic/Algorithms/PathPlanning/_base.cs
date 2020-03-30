using System;
using System.Collections.Generic;
using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    class PathPlanner
    {
        protected static Random Rng = new Random();

        protected Obstacle[] Obstacles;
        protected IKSolver Solver;
        protected bool CollisionCheck;
        protected int MaxTime;

        protected PathPlanner(Obstacle[] obstacles, IKSolver solver, int maxTime, bool collisionCheck)
        {
            Obstacles = obstacles;
            Solver = solver;
            MaxTime = maxTime;
            CollisionCheck = collisionCheck;
        }

        public virtual (List<Vector3>, List<float[]>) Execute(Manipulator agent, Vector3 goal)
        {
            return default;
        }
    }
}