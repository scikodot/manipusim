using System;
using System.Collections.Generic;
using System.Linq;
using Helper;
using WorkEnv;
using GeneticAlgorithm;
using PathGenerator;
using System.Diagnostics;

namespace RoboDraw
{
    class Manager
    {
        public static Manipulator Manip;
        public static Obstacle[] Obstacles;
        public static Point Goal;
        public static List<Point> Path;
        public static List<Point[]> Joints;
        public static Tree Tree;

        public static Dictionary<string, bool> States;

        public static void Execute()
        {
            States = new Dictionary<string, bool>();
            States.Add("Obstacles", false);
            States.Add("Goal", false);
            States.Add("Path", false);
            States.Add("Joints", false);
            States.Add("Tree", false);

            //manipulator
            Manip = new Manipulator
            {
                Links = new double[] { 2, 2, 2, 2 },
                q = Misc.ToRad(new double[] { 45, 15, 15, 15 }),
                Base = Point.Zero
            };

            double MaxStep = 1;
            Manip.StepRanges = new double[Manip.Links.Length, 2];
            for (int i = 0; i < Manip.Links.Length; i++)
            {
                Manip.StepRanges[i, 0] = -MaxStep;
                Manip.StepRanges[i, 1] = MaxStep;
            }

            //obstacles
            Obstacles = new Obstacle[2];
            Point[] obst_data = new Point[360];
            for (int i = 0; i < obst_data.Length; i++)
            {
                obst_data[i] = new Point
                (
                    Math.Cos(i * Math.PI / 180) + 0,
                    Math.Sin(i * Math.PI / 180) + 3.5
                );
            }
            Obstacles[0] = new Obstacle(obst_data);
            for (int i = 0; i < obst_data.Length; i++)
            {
                obst_data[i] = new Point
                (
                    Math.Cos(i * Math.PI / 180) - 2.2,
                    Math.Sin(i * Math.PI / 180) + 0
                );
            }
            Obstacles[1] = new Obstacle(obst_data);
            States["Obstacles"] = true;

            //goal
            Goal = new Point(-2 * Math.Sqrt(2) - 2, 0);
            States["Goal"] = true;

            //initialize genetic algorithm parameters
            Algorithm.Initialize(Manip, Obstacles);

            IKP Solver = new IKP(0.02, Manip.Links.Length, 10, 0.2, 50);
            //AssociativeTable table = new AssociativeTable(Manip, 1000);
            //PathPlanner PathPlanner = new PathPlanner(1, 200, 10, 0.2, 100, Solver);

            Generator.Solver = Solver;
            Tree = Generator.RRT(new Random(), Goal, Manip, Obstacles, 20000, 0.04);
            Tree.Node start = Tree.Min(Goal), node_curr = start;
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
            path.Reverse();
            configs.Reverse();
            States["Tree"] = true;

            Path = path;
            States["Path"] = true;

            Joints = new List<Point[]>();
            foreach (var config in configs)
            {
                if (config == null)
                    break;
                Manip.q = config;
                Joints.Add(Manip.Joints);
            }
            States["Joints"] = true;

            /*var res = PathPlanner.Execute(Goal);
            Path = res.Item3.ToList();
            States["Path"] = true;

            Joints = new List<Point[]>();
            foreach (var config in res.Item4)
            {
                if (config == null)
                    break;
                Manip.q = config;
                Joints.Add(Manip.Joints);
            }
            States["Joints"] = true;*/

            //qualify paths
            /*Generator.Solver = Solver;
            int PathCount = 1;
            for (int i = 0; i < PathCount; i++)
            {
                //generate path
                Path = Generator.GeneratePath(Manip, Obstacles, Goal);
                
                int BadFit = 0;
                if (BadFit == 0)
                {
                    break;
                }
                else
                {
                    //train generator to make better paths
                }
                
                //Console.WriteLine("Bad points: {0}", BadFit);
                //Console.WriteLine("Total time: {0}", Algorithm.TotalRealTime);
            }
            States["Path"] = true;*/
        }
    }
}
