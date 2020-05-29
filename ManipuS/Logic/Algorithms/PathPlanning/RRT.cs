using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Logic.InverseKinematics;
using Physics;

namespace Logic.PathPlanning
{
    public class RRT : PathPlanner
    {
        protected static float _stepDefault = 0.04f;
        protected static bool _showTreeDefault = true;
        protected static bool _discardOutliersDefault = true;
        protected static bool _enableTrimmingDefault = true;
        protected static int _trimPeriodDefault = 1000;

        protected float _step;
        public ref float Step => ref _step;

        protected bool _showTree;
        public ref bool ShowTree => ref _showTree;

        protected bool _discardOutliers;
        public ref bool DiscardOutliers => ref _discardOutliers;

        protected bool _enableTrimming;
        public ref bool EnableTrimming => ref _enableTrimming;

        protected int _trimPeriod;
        public ref int TrimPeriod => ref _trimPeriod;

        public Tree Tree { get; protected set; }

        public RRT(int maxIterations, float threshold, bool collisionCheck, 
            float step, bool showTree, bool discardOutliers, bool enableTrimming, int trimPeriod) : 
            base(maxIterations, threshold, collisionCheck)
        {
            _step = step;
            _threshold = threshold;
            _showTree = showTree;
            _discardOutliers = discardOutliers;
            _enableTrimming = enableTrimming;
            _trimPeriod = trimPeriod;
        }

        public static RRT Default()
        {
            return new RRT(_maxIterationsDefault, _thresholdDefault, _collisionCheckDefault, 
                _stepDefault, _showTreeDefault, _discardOutliersDefault, _enableTrimmingDefault, _trimPeriodDefault);
        }

        protected override PathPlanningResult RunAbstract(Manipulator manipulator, Vector3 goal, InverseKinematicsSolver solver)
        {
            // create new tree
            Tree = new Tree(new Tree.Node(null, manipulator.GripperPos, manipulator.q));

            int iterations = 0;
            while (iterations++ < _maxIterations)
            {
                // trim tree
                if (_enableTrimming && iterations % _trimPeriod == 0)
                    Tree.Trim(manipulator, solver);

                // generate sample
                Vector3 sample = RandomThreadStatic.NextPointSphere(manipulator.WorkspaceRadius) + manipulator.Base;

                // find the closest node to the generated sample point
                Tree.Node nodeClosest = Tree.Closest(sample);

                // get new tree node point
                Vector3 point = nodeClosest.Point + Vector3.Normalize(sample - nodeClosest.Point) * _step;

                if (!(_discardOutliers && ObstacleHandler.ContainmentTest(point, out _)))
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
                    }
                }

                // stop in case the main attractor has been hit
                if (manipulator.GripperPos.DistanceTo(goal) < _threshold)
                    break;
            }

            // retrieve resultant path along with respective configurations
            return new PathPlanningResult
            {
                Iterations = iterations - 1,
                Path = Tree.GetPath(manipulator, Tree.Closest(goal))  // TODO: refactor! tree should be written to temp variable in path planner, not permanent in manipulator
            };
        }
    }
}
