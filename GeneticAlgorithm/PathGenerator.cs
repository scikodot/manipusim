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

        public static Tree RRT(Random rng, Point goal, Manipulator manip, Obstacle[] obstacles, int k, double d)
        {
            Agent = new Manipulator(manip);
            Obstacles = obstacles;
            //for (int i = 0; i < Algorithm.Agent.StepRanges.GetLength(0); i++)
            //{
            //    for (int j = 0; j < Algorithm.Agent.StepRanges.GetLength(1); j++)
            //    {
            //        Algorithm.Agent.StepRanges[i, j] *= 2;
            //    }
            //}

            Tree tree = new Tree(new Tree.Node(null, Agent.GripperPos, Agent.q));

            for (int i = 0; i < k; i++)
            {
                if ((i + 1) % 5000 == 0)
                {
                    tree.RectifyWhole();
                }

                double work_radius;
                double x;
                double y_plus, y_minus;
                double y;

                /*bool GoalConvergence = false;
                if (!GoalConvergence)
                {
                    work_radius = Agent.Links.Sum();
                    x = Agent.Base.x + rng.NextDouble() * 2 * work_radius - work_radius;
                    y_plus = Math.Sqrt(work_radius * work_radius - x * x);
                    y_minus = -y_plus;
                    y = Agent.Base.y + (rng.NextDouble() * 2 * (y_plus - y_minus) - (y_plus - y_minus)) / 2;
                    if ((i + 1) % 800 == 0)
                        GoalConvergence = true;
                }
                else
                {
                    work_radius = 1;
                    x = goal.x + rng.NextDouble() * 2 * work_radius - work_radius;
                    y_plus = Math.Sqrt(work_radius * work_radius - x * x);
                    y_minus = -y_plus;
                    y = goal.y + (rng.NextDouble() * 2 * (y_plus - y_minus) - (y_plus - y_minus)) / 2;
                    if ((i + 1) % 100 == 0)
                        GoalConvergence = false;
                }*/

                /*double[] arr = new double[Manager.AttrWeights.Count];
                Manager.AttrWeights.CopyTo(arr);
                var AttrWeightsLoc = arr.ToList();
                AttrWeightsLoc.Sort();
                
                double num = Misc.BoxMullerTransform(rng, Manager.AttrWeights.Min(), Manager.AttrWeights.Max() / 3);
                double weight = Manager.AttrWeights.Find((t) => { return t > num; });
                if (weight == 0)
                    weight = Manager.AttrWeights.Max();
                int index = Manager.AttrWeights.IndexOf(weight);*/

                int index = rng.Next(0, Manager.Attractors.Count);

                work_radius = Manager.Attractors[index].Radius;
                x = rng.NextDouble() * 2 * work_radius - work_radius;
                y_plus = Math.Sqrt(work_radius * work_radius - x * x);
                y_minus = -y_plus;
                y = Manager.Attractors[index].Center.y + (rng.NextDouble() * 2 * (y_plus - y_minus) - (y_plus - y_minus)) / 2;

                Point p = new Point(x + Manager.Attractors[index].Center.x, y);
                Tree.Node min_node = tree.Min(p);

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
                    Algorithm.Agent.q = min_node.q;
                    var res = Solver.Execute(p_n);
                    if (res.Item1 && !res.Item4.Contains(true))
                    {
                        if (Manager.Attractors[index].InliersCount < 5)
                        {
                            tree.AddNode(new Tree.Node(min_node, p_n, Algorithm.Agent.q.Zip(res.Item3, (t, s) => { return t + s; }).ToArray()));
                            if (p_n.DistanceTo(Manager.Attractors[index].Center) < Manager.Attractors[index].Radius)
                                Manager.Attractors[index].InliersCount++;
                        }
                        else
                        {
                            Manager.Attractors.RemoveAt(index);
                        }
                    }
                }
            }

            return tree;
        }
    }
}
