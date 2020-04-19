using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    class DynamicRRT : PathPlanner
    {
        private float d;
        private int period;

        public DynamicRRT(int maxTime, bool collisionCheck, float d, int period) : base(maxTime, collisionCheck)
        {
            this.d = d;
            this.period = period;
        }

        public override (List<Vector3>, List<Vector>) Execute(Obstacle[] Obstacles, Manipulator agent, Vector3 goal, IKSolver Solver)
        {
            var contestant = agent.DeepCopy();

            // creating new tree
            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));

            // sorting attractors for easier work
            var attractors = new List<Attractor>(agent.Attractors);
            attractors.Sort((t, s) => t.Weight <= s.Weight ? (t.Weight < s.Weight ? -1 : 0) : 1);

            for (int i = 0; i < MaxTime; i++)
            {
                //if (i % period == 0 && i != 0)
                //    agent.Tree.Trim(Obstacles, contestant, Solver);

                // generating normally distributed value with Box-Muller transform
                float num = ProbabilityTheory.RandomGaussian(attractors[0].Weight, (attractors[attractors.Count - 1].Weight - attractors[0].Weight) / 3);  // TODO: check distribution!

                // extracting the index of the most relevant attractor
                int index = attractors.NearestIndex(num, a => a.Weight);

                float radius = attractors[index].Radius, x, yPos, y, zPos, z;

                // generating point of attraction (inside the attractor's field) for tree
                x = -radius + (float)Rng.NextDouble() * 2 * radius;
                yPos = (float)Math.Sqrt(radius * radius - x * x);
                y = -yPos + (float)Rng.NextDouble() * 2 * yPos;
                zPos = (float)Math.Sqrt(radius * radius - x * x - y * y);
                z = -zPos + (float)Rng.NextDouble() * 2 * zPos;

                Vector3 p = new Vector3(x, y, z) + attractors[index].Center;

                // finding the closest node to the generated point
                Tree.Node minNode = agent.Tree.Min(p);

                // creating offset vector to new node
                Vector3 pNew = minNode.Point + Vector3.Normalize(p - minNode.Point) * d;

                bool isClose = pNew.DistanceTo(attractors[index].Center) < attractors[index].Radius;
                if (isClose && index != 0)
                {
                    // removing attractor if it has been hit
                    attractors.RemoveAt(index);
                }
                else
                {
                    // checking for collisions of the new node
                    bool collision = false;
                    if (CollisionCheck)
                    {
                        foreach (var obst in Obstacles)
                        {
                            if (obst.Contains(pNew))
                            {
                                collision = true;
                                break;
                            }
                        }
                    }

                    if (!collision)
                    {
                        // solving IKP for new node
                        contestant.q = minNode.q;
                        var res = Solver.Execute(Obstacles, contestant, pNew, contestant.Joints.Length - 1);
                        if (res.Item1 && !(CollisionCheck && res.Item4.Contains(true)))
                        {
                            // adding node to the tree
                            Tree.Node node = new Tree.Node(minNode, contestant.GripperPos, contestant.q);
                            agent.Tree.AddNode(node);

                            // check for exit condition
                            if (isClose && index == 0)
                                attractors[0].InliersCount++;
                        }
                    }
                }

                // stopping in case the main attractor has been hit
                if (attractors[0].InliersCount != 0)
                    break;
            }

            // retrieving resultant path along with respective configurations
            Tree.Node start = agent.Tree.Min(agent.Goal);

            List<Vector3> path = agent.Tree.TraversePath(start).Reverse().ToList();
            List<Vector> configs = agent.Tree.TraverseConfigs(start).Reverse().ToList();

            return (path, configs);
        }
    }
}