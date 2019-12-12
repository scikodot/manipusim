using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Helper;
using WorkEnv;
using GeneticAlgorithm;
using RoboDraw;

namespace PathGenerator
{
    class Generator
    {
        public static Manipulator Agent;
        public static Obstacle[] Obstacles;
        public static IKP Solver;

        public static void RRT(Random rng, ref Tree holder, Point goal, Manipulator manip, Obstacle[] obstacles, int k, double d)
        {
            Agent = new Manipulator(manip);
            Obstacles = obstacles;

            holder = new Tree(new Tree.Node(null, Agent.GripperPos, Agent.q));

            Attractor[] arr = new Attractor[Manager.Attractors.Count];
            Manager.Attractors.CopyTo(arr);
            var AttractorsLoc = arr.ToList();
            AttractorsLoc.Sort((t, s) => { return t.Weight <= s.Weight ? (t.Weight < s.Weight ? -1 : 0) : 1; });

            for (int i = 0; i < k; i++)
            {
                /*if ((i + 1) % 5000 == 0)
                {
                    tree.RectifyWhole();
                }*/

                double num = Misc.BoxMullerTransform(rng, AttractorsLoc[0].Weight, AttractorsLoc[AttractorsLoc.Count / 2 - 1].Weight / 3);
                Attractor attr = AttractorsLoc.Find((t) => { return t.Weight > num; });
                int index = 0;
                if (attr == null)
                    index = rng.Next(AttractorsLoc.Count / 2 - 1, AttractorsLoc.Count);
                else
                    index = AttractorsLoc.IndexOf(attr);

                //int index = rng.Next(0, Manager.Attractors.Count);

                double radius = AttractorsLoc[index].Radius, x, y_pos, y, z_pos, z;
                
                x = -radius + rng.NextDouble() * 2 * radius;
                y_pos = Math.Sqrt(radius * radius - x * x);
                y = -y_pos + rng.NextDouble() * 2 * y_pos;
                z_pos = Math.Sqrt(radius * radius - x * x - y * y);
                z = -z_pos + rng.NextDouble() * 2 * z_pos;

                Point p = new Point(x, y, z) + AttractorsLoc[index].Center;
                Tree.Node min_node = holder.Min(p);

                Vector v = new Vector(min_node.p, p);
                Point p_n = min_node.p + v.Normalized * d;
                bool collision = false;
                foreach (var obst in Obstacles)
                {
                    if (obst.Contains(p_n))
                    {
                        collision = true;
                        break;
                    }
                }

                if (!collision)
                {
                    Algorithm.Agent.q = Misc.CopyArray(min_node.q);
                    var res = Solver.Execute(p_n);
                    if (res.Item1 && !res.Item4.Contains(true))
                    {
                        if (AttractorsLoc[index].InliersCount < 5)
                        {
                            Tree.Node node = new Tree.Node(min_node, p_n, Algorithm.Agent.q);
                            holder.AddNode(node);
                            if (p_n.DistanceTo(AttractorsLoc[index].Center) < AttractorsLoc[index].Radius)
                                AttractorsLoc[index].InliersCount++;

                            Manager.Buffer.Add(node);
                        }
                        else
                        {
                            AttractorsLoc.RemoveAt(index);
                        }
                    }
                }

                if (AttractorsLoc[0].InliersCount != 0)
                    break;
            }
        }
    }
}
