using System;
using System.Collections.Generic;
using System.Linq;

namespace Logic
{
    class PathPlanner
    {
        public static void RRT(Random rng, Manipulator agent, Obstacle[] obstacles, HillClimbing solver, int k, double d)
        {
            Manipulator Contestant = new Manipulator(agent);

            // creating new tree
            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));

            // sorting attractors for easier work
            var AttractorsLoc = new List<Attractor>(agent.Attractors);
            AttractorsLoc.Sort((t, s) => { return t.Weight <= s.Weight ? (t.Weight < s.Weight ? -1 : 0) : 1; });

            for (int i = 0; i < k; i++)
            {
                // generating normally distributed value with Box-Muller transform
                double num = Misc.BoxMullerTransform(rng, AttractorsLoc[0].Weight, AttractorsLoc[AttractorsLoc.Count - 1].Weight / 3);

                // extracting the first relevant attractor
                Attractor attr = AttractorsLoc.Find((t) => { return t.Weight > num; });
                
                int index = 0;
                if (attr == null)  // clamping weight
                    index = AttractorsLoc.Count - 1;
                else
                    index = AttractorsLoc.IndexOf(attr);

                double radius = AttractorsLoc[index].Radius, x, y_pos, y, z_pos, z;

                // generating point of attraction (inside the attractor's field) for tree
                x = -radius + rng.NextDouble() * 2 * radius;
                y_pos = Math.Sqrt(radius * radius - x * x);
                y = -y_pos + rng.NextDouble() * 2 * y_pos;
                z_pos = Math.Sqrt(radius * radius - x * x - y * y);
                z = -z_pos + rng.NextDouble() * 2 * z_pos;

                Point p = new Point(x, y, z) + AttractorsLoc[index].Center;

                // finding the closest node to the generated point
                Tree.Node minNode = agent.Tree.Min(p);

                // creating offset vector to new node
                Vector v = new Vector(minNode.p, p);
                Point p_n = minNode.p + v.Normalized * d;

                // checking for collisions of the new node
                bool collision = false;
                foreach (var obst in obstacles)
                {
                    if (obst.Contains(p_n))
                    {
                        collision = true;
                        break;
                    }
                }

                if (!collision)
                {
                    // solving IKP for new node
                    Contestant.q = Misc.CopyArray(minNode.q);
                    var res = solver.Execute(Contestant, p_n);
                    if (res.Item1 && !res.Item4.Contains(true))
                    {
                        // adding node to the tree
                        Tree.Node node = new Tree.Node(minNode, p_n, Contestant.q);
                        agent.Tree.AddNode(node);
                        if (p_n.DistanceTo(AttractorsLoc[index].Center) < AttractorsLoc[index].Radius)
                        {
                            // removing attractor if it has been hit
                            if (index != 0)
                                AttractorsLoc.RemoveAt(index);
                            else
                                AttractorsLoc[index].InliersCount++;
                        }

                        agent.Tree.Buffer.Add(node);
                    }
                }

                // stopping in case the main attractor has been hit
                if (AttractorsLoc[0].InliersCount != 0)
                    break;
            }
        }
    }
}
