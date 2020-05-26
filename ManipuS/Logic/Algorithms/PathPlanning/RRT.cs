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
        protected static float _thresholdDefault = 0.04f;
        protected static bool _showTreeDefault = true;
        protected static bool _discardOutliersDefault = true;

        protected float _step;
        public ref float Step => ref _step;

        protected float _threshold;
        public ref float Threshold => ref _threshold;

        protected bool _showTree;
        public ref bool ShowTree => ref _showTree;

        protected bool _discardOutliers;
        public ref bool DiscardOutliers => ref _discardOutliers;

        public Tree Tree { get; protected set; }

        public RRT(int maxIterations, bool collisionCheck, float step, float threshold, bool showTree, bool discardOutliers) : 
            base(maxIterations, collisionCheck)
        {
            _step = step;
            _threshold = threshold;
            _showTree = showTree;
            _discardOutliers = discardOutliers;
        }

        public static RRT Default()
        {
            return new RRT(_maxIterationsDefault, _collisionCheckDefault, _stepDefault, _thresholdDefault, 
                _showTreeDefault, _discardOutliersDefault);
        }

        protected override (int, Path) RunAbstract(Manipulator manipulator, Vector3 goal, InverseKinematicsSolver solver)
        {
            // create new tree
            Tree = new Tree(new Tree.Node(null, manipulator.GripperPos, manipulator.q));

            int iters = 0;
            while (iters++ < _maxIterations)
            {
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
                    (var converged, _, var distance, var offset) = solver.Execute(manipulator, point, manipulator.Joints.Length - 1);
                    if (converged && !(_collisionCheck && manipulator.CollisionTest().Contains(true)))
                    {
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
            return (iters - 1, Tree.GetPath(manipulator, Tree.Closest(goal)));  // TODO: refactor! tree should be written to temp variable in path planner, not permanent in manipulator
        }
    }
}
