using System;
using System.Numerics;

using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    public enum PathPlannerType  // TODO: for scalability enum should be replaced with dictionary (?) to enable adding custom derived classes
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
        protected static int _maxIterationsDefault = 10000;
        protected static bool _collisionCheckDefault = true;

        public static string[] Types { get; } = Enum.GetNames(typeof(PathPlannerType));

        public PathPlannerType Type { get; private set; }

        protected int _maxIterations;
        public ref int MaxIterations => ref _maxIterations;

        protected bool _collisionCheck;
        public ref bool CollisionCheck => ref _collisionCheck;

        protected PathPlanner(int maxTime, bool collisionCheck)
        {
            _maxIterations = maxTime;
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