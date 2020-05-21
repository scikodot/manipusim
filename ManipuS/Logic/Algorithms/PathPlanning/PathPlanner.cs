using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    public enum PathPlannerType
    {
        RRT,
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

    public abstract class PathPlanner
    {
        public static string[] Types { get; } = Enum.GetNames(typeof(PathPlannerType));

        protected static Random Rng { get; } = new Random();

        protected bool CollisionCheck;

        protected int _maxTime;
        public ref int MaxTime => ref _maxTime;

        public int Iterations;

        protected PathPlanner(int maxTime, bool collisionCheck)
        {
            MaxTime = maxTime;
            CollisionCheck = collisionCheck;
        }

        public abstract Path Execute(Obstacle[] Obstacles, Manipulator agent, Vector3 goal, InverseKinematicsSolver Solver);
    }
}