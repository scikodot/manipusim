using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Logic
{
    class PathPlanner
    {
        public static void RRT(Random rng, Manipulator agent, Obstacle[] obstacles, IKP solver, int k, double d)
        {
            Manipulator Contestant = new Manipulator(agent);

            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));

            var AttractorsLoc = new List<Attractor>(agent.Attractors);
            AttractorsLoc.Sort((t, s) => { return t.Weight <= s.Weight ? (t.Weight < s.Weight ? -1 : 0) : 1; });
            //Attractor[] arr = new Attractor[manip.Attractors.Count];
            //manip.Attractors.CopyTo(arr);
            //var AttractorsLoc = arr.ToList();
            //AttractorsLoc.Sort((t, s) => { return t.Weight <= s.Weight ? (t.Weight < s.Weight ? -1 : 0) : 1; });

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
                Tree.Node min_node = agent.Tree.Min(p);

                Vector v = new Vector(min_node.p, p);
                Point p_n = min_node.p + v.Normalized * d;
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
                    Contestant.q = Misc.CopyArray(min_node.q);
                    var res = solver.Execute(Contestant, p_n);
                    if (res.Item1 && !res.Item4.Contains(true))
                    {
                        if (AttractorsLoc[index].InliersCount < 5)
                        {
                            Tree.Node node = new Tree.Node(min_node, p_n, Contestant.q);
                            agent.Tree.AddNode(node);
                            if (p_n.DistanceTo(AttractorsLoc[index].Center) < AttractorsLoc[index].Radius)
                                AttractorsLoc[index].InliersCount++;

                            agent.Tree.Buffer.Add(node);
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
