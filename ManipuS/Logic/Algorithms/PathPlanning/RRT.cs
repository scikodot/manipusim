using System;
using System.Collections.Generic;
using System.Linq;

namespace Logic
{
    static partial class PathPlanner
    {
        public static (List<Point>, List<double[]>) RRT(Manipulator agent, Obstacle[] obstacles, HillClimbing solver, int k, double d, bool collisionCheck)  // TODO: all algorithms should be placed in separate files!
        {
            Manipulator Contestant = new Manipulator(agent);

            // creating new tree
            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));

            // sorting attractors for easier work
            var Attractors = new List<Attractor>(agent.GoodAttractors);
            Attractors.Sort((t, s) => { return t.Weight <= s.Weight ? (t.Weight < s.Weight ? -1 : 0) : 1; });

            for (int i = 0; i < k; i++)
            {
                // generating normally distributed value with Box-Muller transform
                double num = Misc.BoxMullerTransform(Rng, Attractors[0].Weight, (Attractors[Attractors.Count - 1].Weight - Attractors[0].Weight) / 3);  // TODO: check distribution!

                // extracting the first relevant attractor
                Attractor attr = Attractors.Find((t) => { return t.Weight > num; });
                
                int index = 0;
                if (attr == null)  // clamping weight
                    index = Attractors.Count - 1;
                else
                    index = Attractors.IndexOf(attr);

                double radius = Attractors[index].Radius, x, y_pos, y, z_pos, z;

                // generating point of attraction (inside the attractor's field) for tree
                x = -radius + Rng.NextDouble() * 2 * radius;
                y_pos = Math.Sqrt(radius * radius - x * x);
                y = -y_pos + Rng.NextDouble() * 2 * y_pos;
                z_pos = Math.Sqrt(radius * radius - x * x - y * y);
                z = -z_pos + Rng.NextDouble() * 2 * z_pos;

                Point p = new Point(x, y, z) + Attractors[index].Center;

                // finding the closest node to the generated point
                Tree.Node minNode = agent.Tree.Min(p);

                // creating offset vector to new node
                Vector v = new Vector(minNode.p, p);
                Point pNew = minNode.p + v.Normalized * d;

                // checking for collisions of the new node
                bool collision = false;
                if (collisionCheck)
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
                    Contestant.q = Misc.CopyArray(minNode.q);
                    var res = solver.Execute(Contestant, pNew);
                    if (res.Item1 && !(collisionCheck && res.Item4.Contains(true)))
                    {                            
                        // adding node to the tree
                        Tree.Node node = new Tree.Node(minNode, Contestant.GripperPos, Contestant.q);
                        agent.Tree.AddNode(node);
                        if (pNew.DistanceTo(Attractors[index].Center) < Attractors[index].Radius)
                        {
                            // removing attractor if it has been hit
                            if (index != 0)
                                Attractors.RemoveAt(index);
                            else
                                Attractors[index].InliersCount++;
                        }

                        agent.Tree.Buffer.Add(node);
                    }
                }

                // stopping in case the main attractor has been hit
                if (Attractors[0].InliersCount != 0)
                    break;
            }

            // retrieving resultant path along with respective configurations
            Tree.Node start = agent.Tree.Min(agent.Goal), node_curr = start;
            List<Point> path = new List<Point>();
            List<double[]> configs = new List<double[]>();
            for (int i = start.Layer; i >= 0; i--)
            {
                if (node_curr.Layer == i)
                {
                    path.Add(node_curr.p);
                    configs.Add(node_curr.q);
                    if (node_curr.Parent != null)
                    {
                        int pointsNum = node_curr.Layer - node_curr.Parent.Layer - 1;
                        if (pointsNum > 0)
                        {
                            Tree.Node[] nodes = Tree.Discretize(node_curr, node_curr.Parent, pointsNum);
                            foreach (var node in nodes)
                            {
                                configs.Add(node.q);
                            }
                        }
                    }

                    node_curr = node_curr.Parent;
                }
            }

            // reverting path so that it goes from root to goal
            path.Reverse();
            configs.Reverse();

            return (path, configs);
        }
    }
}
