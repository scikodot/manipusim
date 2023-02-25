using System;
using System.Linq;

using BulletSharp.Math;

using System.Threading;
using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    public class RRT : PathPlanner
    {
        protected static bool _showTreeDefault = true;  // TODO: make constants?
        protected static bool _enableTrimmingDefault = false;
        protected static float _stepDefault = 0.04f;
        protected static int _trimPeriodDefault = 1000;
        protected const int _goalBiasPeriodDefault = 50;

        protected float _step;
        public ref float Step => ref _step;

        protected bool _showTree;
        public ref bool ShowTree => ref _showTree;  // TODO: move to model!

        protected bool _enableTrimming;
        public ref bool EnableTrimming => ref _enableTrimming;

        protected int _trimPeriod;
        public ref int TrimPeriod => ref _trimPeriod;

        protected int _goalBiasPeriod;
        public ref int GoalBiasPeriod => ref _goalBiasPeriod;

        public Tree Tree { get; protected set; }

        public RRT(int maxIterations, float threshold, bool collisionCheck, 
            float step, bool showTree, bool enableTrimming, int trimPeriod, int goalBiasPeriod = _goalBiasPeriodDefault) : 
            base(maxIterations, threshold, collisionCheck)
        {
            _threshold = threshold;
            _step = step;
            _showTree = showTree;
            _enableTrimming = enableTrimming;
            _trimPeriod = trimPeriod;
            _goalBiasPeriod = goalBiasPeriod;
        }

        public static RRT Default()
        {
            return new RRT(_maxIterationsDefault, _thresholdDefault, _collisionCheckDefault, 
                _stepDefault, _showTreeDefault, _enableTrimmingDefault, _trimPeriodDefault);
        }

        protected override PathPlanningResult RunAbstract(Manipulator manipulator, Vector3 goal, InverseKinematicsSolver solver, CancellationToken cancellationToken)
        {
            if (manipulator.DistanceTo(goal) < _threshold)
                // the goal is already reached
                return new PathPlanningResult
                {
                    Iterations = 0,
                    Path = null
                };

            // create new tree
            Tree = new Tree(new Tree.Node(null, manipulator.GripperPos, manipulator.q), _maxIterations + 1);

            Iterations = 0;
            while (Iterations < _maxIterations)
            {
                cancellationToken.ThrowIfCancellationRequested();

                Iterations++;

                // trim tree
                if (_enableTrimming && Iterations % _trimPeriod == 0)
                    Tree.Trim(manipulator, solver);

                // get sample point
                Vector3 sample;
                if (Iterations % _goalBiasPeriod == 0)
                    // use goal bias
                    sample = goal;
                else
                    // generate sample
                    sample = RandomThreadStatic.NextPointSphere(manipulator.WorkspaceRadius) + manipulator.Base;

                // find the closest node to the generated sample point
                Tree.Node nodeClosest = Tree.Closest(sample);

                // get new tree node point
                Vector3 point = nodeClosest.Point + Vector3.Normalize(sample - nodeClosest.Point) * _step;

                if (!(_collisionCheck /*&& ObstacleHandler.ContainmentTest(point, out _)*/))
                {
                    // solve inverse kinematics for the new node to obtain the agent configuration
                    manipulator.q = nodeClosest.q;
                    var ikRes = solver.Execute(manipulator, point);
                    if (ikRes.Converged && !(_collisionCheck && manipulator.CollisionTest().Contains(true)))
                    {
                        manipulator.q = ikRes.Configuration;

                        // add new node to the tree
                        Tree.Node node = new Tree.Node(nodeClosest, manipulator.GripperPos, manipulator.q);
                        Tree.AddNode(node);

                        // check exit condition
                        if (manipulator.DistanceTo(goal) < _threshold)
                            break;
                    }
                }
            }

            // retrieve resultant path along with respective configurations
            return new PathPlanningResult
            {
                Iterations = Iterations,
                Path = Tree.GetPath(manipulator, Tree.Closest(goal))  // TODO: refactor! tree should be written to temp variable in path planner, not permanent in manipulator
            };
        }

        protected override void Reset()
        {
            Tree = null;
        }
    }
}
