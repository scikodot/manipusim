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
        ARRT,
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

        public PathPlannerType Type { get; private set; }

        protected PathPlanner(int maxTime, bool collisionCheck)
        {
            _maxTime = maxTime;
            _collisionCheck = collisionCheck;

            Type = (PathPlannerType)Enum.Parse(typeof(PathPlannerType), GetType().Name);
        }

        public (int, Path) Run(Manipulator manipulator, Vector3 goal, InverseKinematicsSolver solver)
        {
            using (var manipulatorCopy = manipulator.DeepCopy())
            {
                return RunAbstract(manipulatorCopy, goal, solver);
            }
        }

        protected abstract (int, Path) RunAbstract(Manipulator manipulator, Vector3 goal, InverseKinematicsSolver solver);
    }
}