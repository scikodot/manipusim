using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    public enum PathPlannerType
    {
        DynamicRRT,
        GeneticAlgorithm
    }

    public struct PathPlanningData
    {
        public int PathPlannerID;
        public int AttrNum;
        public int k;
        public float d;
    }

    public class PathPlanner
    {
        public static string[] Types = Dispatcher.GetDerivedTypes(typeof(PathPlanner)).ToArray();

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