using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Helper;
using WorkEnv;
using GeneticAlgorithm;

namespace PathGenerator
{
    class Generator
    {
        public static Manipulator Agent;
        public static Obstacle[] Obstacles;
        public static IKP Solver;

        public static List<Point> GeneratePath(Manipulator agent, Obstacle[] obstacles, Point goal)
        {
            Random rng = new Random();
            Agent = agent;
            Obstacles = obstacles;

            List<Point> Path = new List<Point>();

            Point gripperPos = agent.GripperPos;
            Vector dir = new Vector(gripperPos, goal);

            int Time = 0;
            for (int i = 0; i < 500; i++)
            {
                //generate vectors
                Vector[] vecs = new Vector[50];
                for (int j = 0; j < 50; j++)
                {
                    double angle = (rng.NextDouble() * 2 * Math.PI - Math.PI) + dir.Angle;
                    vecs[j] = new Vector(Math.Cos(angle), Math.Sin(angle)) * 0.02;
                }

                //qualification
                var obj_features = new List<double>();
                var obst_features = new List<bool>();
                var manip_features = new List<bool>();
                var configs = new List<double[]>();
                foreach (var vec in vecs)
                {
                    var res = Qualify(vec, goal, gripperPos, obstacles);
                    obj_features.Add(res.Item1);
                    obst_features.Add(res.Item2);
                    manip_features.Add(res.Item3);
                    configs.Add(res.Item4);
                }

                //retrieving dominant vector, if there's any
                double min = double.MaxValue;
                int min_index = -1;
                for (int j = 0; j < obst_features.Count; j++)
                {
                    if (!obst_features[j] && manip_features[j])
                    {
                        if (obj_features[j] < min)
                        {
                            min = obj_features[j];
                            min_index = j;
                        }
                    }
                }
                
                if (min_index != -1)
                {
                    gripperPos += vecs[min_index];
                    dir = new Vector(gripperPos, goal);
                    Path.Add(gripperPos);
                    Agent.q = Agent.q.Zip(configs[min_index], (t, s) => { return t + s; }).ToArray();
                }

                Time++;

                //quitting if the gripper is near the goal
                if (gripperPos.DistanceTo(goal) < 0.15)
                    break;
            }

            foreach (var p in Path)
            {
                Console.WriteLine("({0}, {1})", p.x, p.y);
            }
            Console.WriteLine("Time taken: {0}\n", Time);

            return Path;
        }

        public static Tuple<double, bool, bool, double[]> Qualify(Vector vec, Point goal, Point gripperPos, Obstacle[] obstacles)
        {
            Point pos = gripperPos + vec;

            double obj_feature;
            bool obst_feature = false, manip_feature = true;

            //obstacle criteria
            foreach (var obst in obstacles)
            {
                if (obst.Contains(pos))
                    obst_feature = true;
            }

            //object criteria
            obj_feature = Vector.AngleBetween(new Vector(gripperPos, goal), vec);

            //manipulator criteria
            Algorithm.Agent.q = Agent.q;
            var res = Solver.Execute(pos);
            if ((res.Item1 || res.Item2 <= 2 * IKP.Precision) && !res.Item4.Contains(true))
            {
                manip_feature = true;
            }
            else
            {
                manip_feature = false;
            }

            return new Tuple<double, bool, bool, double[]>(obj_feature, obst_feature, manip_feature, res.Item3);
        }

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
                if ((i + 1) % 1000 == 0)
                {
                    tree.RectifyWhole();
                }

                double work_radius;
                double x;
                double y_plus, y_minus;
                double y;

                bool GoalConvergence = false;
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
                }

                Point p = new Point(x, y);
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
                        tree.AddNode(new Tree.Node(min_node, p_n, Algorithm.Agent.q.Zip(res.Item3, (t, s) => { return t + s; }).ToArray()));
                    }
                }
            }

            /*Agent = new Manipulator(manip);
            Obstacles = obstacles;

            Tree tree = new Tree(new Tree.Node(null, Agent.GripperPos, Agent.q));

            for (int i = 0; i < k; i++)
            {
                double[] dq = new double[Agent.Links.Length];
                for (int j = 0; j < Agent.Links.Length; j++)
                {
                    dq[j] = manip.StepRanges[j, 0] + (manip.StepRanges[j, 1] - manip.StepRanges[j, 0]) * rng.NextDouble();
                    dq[j] *= Math.PI / 180;
                }

                //int layer_index = rng.Next(0, tree.Layers.Count);
                int node_index = rng.Next(0, tree.Layers[tree.Layers.Count - 1].Count);
                Tree.Node parent = tree.Layers[tree.Layers.Count - 1][node_index];
                while (parent.Childs.Count > 2)
                {
                    //layer_index = rng.Next(0, tree.Layers.Count);
                    node_index = rng.Next(0, tree.Layers[tree.Layers.Count - 1].Count);
                    parent = tree.Layers[tree.Layers.Count - 1][node_index];
                }

                Agent.q = parent.q.Zip(dq, (t, s) => { return t + s; }).ToArray();
                bool[] collisions = Algorithm.DetectCollisions(Agent);

                if (!collisions.Contains(true))
                {
                    tree.AddNode(new Tree.Node(parent, Agent.GripperPos, Agent.q));
                }
            }*/

            return tree;
        }
    }
}
