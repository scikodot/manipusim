using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using BulletSharp;
using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    class DynamicRRT : PathPlanner
    {
        private float _step;
        public ref float Step => ref _step;

        private int _trimPeriod;
        public ref int TrimPeriod => ref _trimPeriod;

        public DynamicRRT(int maxTime, bool collisionCheck, float step, int trimPeriod) : base(maxTime, collisionCheck)
        {
            _step = step;
            _trimPeriod = trimPeriod;
        }

        public override (List<Vector3>, List<VectorFloat>) Execute(Obstacle[] obstacles, Manipulator agent, Vector3 goal, InverseKinematicsSolver solver)
        {
            var contestant = agent.DeepCopy();

            // creating new tree
            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));  // TODO: consider creating local tree and returning it (for benchmarking or similar)

            // sorting attractors for easier work
            var attractors = new List<Attractor>(agent.Attractors);
            attractors = attractors.OrderBy(a => a.Weight).ToList();

            // create necessary locals
            var attractorFirst = attractors.First();
            var attractorLast = attractors.Last();  // TODO: last attractor has too big radius (~5 units for 0.04 step); fix!

            float mu = attractorFirst.Weight;
            float sigma = (attractorLast.Weight - attractorFirst.Weight) / 3.0f;  // TODO: check distribution!

            for (int i = 0; i < MaxTime; i++)  // TODO: rename to TimeLimit?
            {
                //if (i % _trimPeriod == 0 && i != 0)
                //    agent.Tree.Trim(obstacles, contestant, solver);

                // generating normally distributed weight
                float num = RandomThreadStatic.NextGaussian(mu, sigma);

                // extracting the index of the most relevant attractor
                int index = attractors.NearestIndex(num, a => a.Weight);

                Vector3 p = attractors[index].Center;

                // finding the closest node to the generated point
                Tree.Node minNode = agent.Tree.Min(p);

                // creating offset vector to new node
                Vector3 pNew = minNode.Point + Vector3.Normalize(p - minNode.Point) * _step;

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
                        foreach (var obst in obstacles)
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
                        var res = solver.Execute(obstacles, contestant, pNew, contestant.Joints.Length - 1);
                        if (res.Item1 && !(CollisionCheck && res.Item4.Contains(true)))
                        {
                            // adding node to the tree
                            Tree.Node node = new Tree.Node(minNode, contestant.GripperPos, contestant.q);
                            agent.Tree.AddNode(node);

                            // check for exit condition
                            if (isClose && index == 0)
                                attractorFirst.InliersCount++;
                        }
                    }
                }

                // stopping in case the main attractor has been hit
                //if (attractors[0].InliersCount != 0)
                //    break;
            }

            // retrieving resultant path along with respective configurations
            Tree.Node start = agent.Tree.Min(agent.Goal);

            List<Vector3> path = agent.Tree.TraversePath(start).Reverse().ToList();
            List<VectorFloat> configs = agent.Tree.TraverseConfigs(start).Reverse().ToList();

            return (path, configs);
        }
    }
}