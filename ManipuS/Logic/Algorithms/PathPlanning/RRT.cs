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
        protected float _step;
        public ref float Step => ref _step;

        protected float _threshold;
        public ref float Threshold => ref _threshold;  // TODO: use!

        public RRT(int maxTime, bool collisionCheck, float step) : base(maxTime, collisionCheck)
        {
            _step = step;
        }

        public override Path Execute(Obstacle[] obstacles, Manipulator agent, Vector3 goal, InverseKinematicsSolver solver)
        {
            Manipulator agentCopy = agent.DeepCopy();

            // create new tree
            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));  // TODO: consider creating local tree and returning it (for benchmarking or similar)

            for (int i = 0; i < MaxTime; i++)
            {
                // generate sample
                Vector3 sample = RandomThreadStatic.NextPoint3D(agent.WorkspaceRadius);  // TODO: the base position is not taken into account; fix!

                // find the closest node to the generated sample point
                Tree.Node nodeClosest = agent.Tree.Closest(sample);

                // get new tree node point
                Vector3 point = nodeClosest.Point + Vector3.Normalize(sample - nodeClosest.Point) * _step;

                if (!IsOutlier(point))
                {
                    // solve inverse kinematics for the new node to obtain the agent configuration
                    agentCopy.q = nodeClosest.q;
                    (var converged, var distance, var offset) = solver.Execute(agentCopy, point, agentCopy.Joints.Length - 1);
                    if (converged && !(CollisionCheck && agent.CollisionTest().Contains(true)))
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
            return agent.Tree.GetPath(agentCopy, agent.Tree.Closest(goal));  // TODO: refactor!
        }

        protected bool IsOutlier(Vector3 point)
        {
            bool isOutlier = false;

            if (CollisionCheck)  // TODO: replace with DiscardOutliers
            {
                foreach (var obstacle in ObstacleHandler.Obstacles)
                {
                    if (obstacle.Contains(point))
                    {
                        isOutlier = true;
                        break;
                    }
                }
            }

            return isOutlier;
        }
    }
}
