using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;

using Logic.InverseKinematics;
using Physics;

namespace Logic.PathPlanning
{
    class DynamicRRT : RRT
    {
        private int _trimPeriod;
        public ref int TrimPeriod => ref _trimPeriod;

        public DynamicRRT(int maxTime, bool collisionCheck, float step, int trimPeriod) : base(maxTime, collisionCheck, step)
        {
            _trimPeriod = trimPeriod;
        }

        public override Path Execute(Obstacle[] obstacles, Manipulator agent, Vector3 goal, InverseKinematicsSolver solver)
        {
            Manipulator agentCopy = agent.DeepCopy();

            // creating new tree
            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));  // TODO: consider creating local tree and returning it (for benchmarking or similar)

            // sorting attractors for easier work
            List<Attractor> attractors = new List<Attractor>(agent.Attractors);
            attractors = attractors.OrderBy(a => a.Weight).ToList();

            // create necessary locals
            Attractor attractorFirst = attractors.First();
            Attractor attractorLast = attractors.Last();  // TODO: last attractor has too big radius (~5 units for 0.04 step); fix!

            float mu = attractorFirst.Weight;
            float sigma = (attractorLast.Weight - attractorFirst.Weight) / 3.0f;  // TODO: check distribution!

            for (int i = 0; i < MaxTime; i++)  // TODO: rename to TimeLimit?
            {
                if (i % _trimPeriod == 0 && i != 0)
                    agent.Tree.Trim(obstacles, agentCopy, solver);

                // generate normally distributed weight
                float num = RandomThreadStatic.NextGaussian(mu, sigma);

                // get the index of the most relevant attractor
                int index = attractors.IndexOfNearest(num, a => a.Weight);

                // generate sample
                Vector3 sample = attractors[index].Center;

                // find the closest node to the generated point
                Tree.Node nodeClosest = agent.Tree.Closest(sample);

                // get new tree node point
                Vector3 point = nodeClosest.Point + Vector3.Normalize(sample - nodeClosest.Point) * _step;

                bool isClose = point.DistanceTo(attractors[index].Center) < attractors[index].Radius;
                if (isClose && index != 0)
                {
                    // remove attractor if it has been hit
                    attractors.RemoveAt(index);
                }
                else
                {
                    if (!IsOutlier(point))
                    {
                        // solve inverse kinematics for the new node
                        agentCopy.q = nodeClosest.q;
                        (var converged, var distance, var offset) = solver.Execute(agentCopy, point, agentCopy.Joints.Length - 1);
                        if (converged && !(CollisionCheck && agent.CollisionTest().Contains(true)))
                        {
                            // add node to the tree
                            Tree.Node node = new Tree.Node(nodeClosest, agentCopy.GripperPos, agentCopy.q);
                            agent.Tree.AddNode(node);

                            // check for exit condition
                            if (isClose && index == 0)
                                attractorFirst.InliersCount++;
                        }
                    }
                }

                // stop in case the main attractor has been hit
                //if (attractorFirst.InliersCount != 0)
                //    break;
            }

            // retrieve resultant path along with respective configurations
            return agent.Tree.GetPath(agentCopy, agent.Tree.Closest(goal));  // TODO: refactor!
        }
    }
}