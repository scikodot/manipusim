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
        protected float _step = 0.04f;
        public ref float Step => ref _step;

        protected float _threshold = 0.04f;
        public ref float Threshold => ref _threshold;

        protected bool _discardOutliers = true;
        public ref bool DiscardOutliers => ref _discardOutliers;

        public Tree Tree { get; protected set; }

        public RRT(int maxTime, bool collisionCheck, float step) : base(maxTime, collisionCheck)
        {
            _step = step;
        }

        protected override (int, Path) RunAbstract(Manipulator agent, Vector3 goal, InverseKinematicsSolver solver)
        {
            Manipulator agentCopy = agent.DeepCopy();

            // create new tree
            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));  // TODO: consider creating local tree and returning it (for benchmarking or similar)

            int iters = 0;
            while (iters++ < _maxTime)
            {
                // generate sample
                Vector3 sample = RandomThreadStatic.NextPointSphere(agent.WorkspaceRadius) + agent.Base;

                // find the closest node to the generated sample point
                Tree.Node nodeClosest = agent.Tree.Closest(sample);

                // get new tree node point
                Vector3 point = nodeClosest.Point + Vector3.Normalize(sample - nodeClosest.Point) * _step;

                if (!(_discardOutliers && ObstacleHandler.ContainmentTest(point)))
                {
                    // solve inverse kinematics for the new node to obtain the agent configuration
                    agentCopy.q = nodeClosest.q;
                    (var converged, _, var distance, var offset) = solver.Execute(agentCopy, point, agentCopy.Joints.Length - 1);
                    if (converged && !(_collisionCheck && agent.CollisionTest().Contains(true)))
                    {
                        // add new node to the tree
                        Tree.Node node = new Tree.Node(nodeClosest, agentCopy.GripperPos, agentCopy.q);
                        agent.Tree.AddNode(node);
                    }
                }

                // stop in case the main attractor has been hit
                if (agentCopy.GripperPos.DistanceTo(goal) < _threshold)
                    break;
            }

            // retrieve resultant path along with respective configurations
            return (iters - 1, agent.Tree.GetPath(agentCopy, agent.Tree.Closest(goal)));  // TODO: refactor! tree should be written to temp variable in path planner, not permanent in manipulator
        }
    }
}
