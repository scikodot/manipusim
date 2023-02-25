using System;
using System.Diagnostics;

using BulletSharp.Math;

using System.Threading;
using System.Threading.Tasks;
using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    public enum PathPlannerType  // TODO: for scalability enum should be replaced with dictionary (?) to enable adding custom derived classes
    {
        RRT,
        ARRT,
        GeneticAlgorithm
    }

    public struct PathPlanningResult  // TODO: add Converged field?
    {
        public int Iterations;
        public Path Path;
    }

    public abstract class PathPlanner
    {
        protected static bool _collisionCheckDefault = true;
        protected static int _maxIterationsDefault = 10000;
        protected static float _thresholdDefault = 0.04f;

        public static string[] Types { get; } = Enum.GetNames(typeof(PathPlannerType));

        public PathPlannerType Type { get; private set; }

        protected int _maxIterations;
        public ref int MaxIterations => ref _maxIterations;

        protected float _threshold;
        public ref float Threshold => ref _threshold;

        protected bool _collisionCheck;
        public ref bool CollisionCheck => ref _collisionCheck;

        public int Iterations { get; protected set; }
        public Stopwatch Timer { get; } = new Stopwatch();
        public ControllerState State { get; private set; } = ControllerState.Idle;

        protected PathPlanner(int maxIterations, float threshold, bool collisionCheck)
        {
            _maxIterations = maxIterations;
            _threshold = threshold;
            _collisionCheck = collisionCheck;

            Type = (PathPlannerType)Enum.Parse(typeof(PathPlannerType), GetType().Name);
        }

        public PathPlanningResult Run(Manipulator manipulator, Vector3 goal, InverseKinematicsSolver solver, CancellationToken cancellationToken = default)
        {
            var res = new PathPlanningResult();

            try
            {
                // restart measuring execution time
                Timer.Restart();

                // turn the controller on
                State = ControllerState.Running;

                // execute path planning
                using (var manipulatorCopy = manipulator.DeepCopy())
                {
                    res = RunAbstract(manipulatorCopy, goal, solver, cancellationToken);
                }

                // turn the controller off
                State = ControllerState.Idle;
            }
            catch (OperationCanceledException oce)
            {
                // TODO: provide some log info
                Reset();

                // indicate that the process has been aborted
                State = ControllerState.Aborted;
            }
            finally
            {
                // stop measuring execution time
                Timer.Stop();
            }

            return res;
        }

        protected abstract PathPlanningResult RunAbstract(Manipulator manipulator, Vector3 goal, InverseKinematicsSolver solver, CancellationToken cancellationToken);

        protected virtual void Reset()
        {

        }
    }
}