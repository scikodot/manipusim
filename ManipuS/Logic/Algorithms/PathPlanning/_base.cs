using System;
using System.Collections.Generic;
using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    public class PathPlanner
    {
        protected static Random Rng = new Random();

        protected bool CollisionCheck;
        protected int MaxTime;

        protected PathPlanner(int maxTime, bool collisionCheck)
        {
            MaxTime = maxTime;
            CollisionCheck = collisionCheck;
        }

        public virtual (List<Vector3>, List<Vector>) Execute(Obstacle[] Obstacles, Manipulator agent, Vector3 goal, IKSolver Solver)
        {
            return default;
        }
    }
}