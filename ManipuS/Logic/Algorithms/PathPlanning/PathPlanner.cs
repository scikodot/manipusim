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

        protected bool _collisionCheck;
        public ref bool CollisionCheck => ref _collisionCheck;

        protected int _maxTime;
        public ref int MaxTime => ref _maxTime;

        protected PathPlanner(int maxTime, bool collisionCheck)
        {
            _maxTime = maxTime;
            _collisionCheck = collisionCheck;
        }

        public abstract (int, Path) Execute(Manipulator agent, Vector3 goal, InverseKinematicsSolver Solver);
    }
}