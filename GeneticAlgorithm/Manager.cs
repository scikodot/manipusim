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
        public static List<Attractor> Attractors;
        public static List<Tree.Node> Buffer = new List<Tree.Node>();

        public static Dictionary<string, bool> States;

        public static void Execute()
        {
            States = new Dictionary<string, bool>
            {
                { "Obstacles", false },
                { "Goal", false },
                { "Path", false },
                { "Joints", false },
                { "Tree", false },
                { "Attractors", false }
            };

            //manipulator
            Manip = new Manipulator
            (
                new Point(-1.5, 0, 1.5),
                new double[] { 0.2, 2, 2, 2 },
                Misc.ToRad(new double[] { 0, 0, 0, 0 }),
                new double[,]
                {
                    { -180, 180 },
                    { -180, 180 },
                    { -180, 180 },
                    { -180, 180 }
                },
                new Tuple<Func<Manipulator, double>, double, double, double>[]
                {
                    new Tuple<Func<Manipulator, double>, double, double, double>((m) => { return m.q[0]; }, 0.2, Math.PI / 2, 0),
                    new Tuple<Func<Manipulator, double>, double, double, double>((m) => { return -Math.PI / 2; }, 0, 0, 0),
                    new Tuple<Func<Manipulator, double>, double, double, double>((m) => { return m.q[1]; }, 0, 0, 2),
                    new Tuple<Func<Manipulator, double>, double, double, double>((m) => { return m.q[2]; }, 0, 0, 2),
                    new Tuple<Func<Manipulator, double>, double, double, double>((m) => { return m.q[3]; }, 0, 0, 2)
                }
            );
            //Manip.DH_Init();
            /*Manip = new Manipulator
            (
                new Point(-1.5, 0, 1.5),
                new double[] { 2, 1, 1, 2, 1, 1 },
                Misc.ToRad(new double[] { 15, 15, 45, 15, 15, 45 }),
                new double[,]
                {
                    { -180, 180 },
                    { -180, 180 },
                    { -180, 180 },
                    { -180, 180 },
                    { -180, 180 },
                    { -180, 180 }
                },
                new double[][]
                {
                    new double[] { 2, Math.PI / 2, 0 },
                    new double[] { 0, -Math.PI / 2, 0 },
                    new double[] { 2, Math.PI / 2, 0 },
                    new double[] { 0, -Math.PI / 2, 0 },
                    new double[] { 2, Math.PI / 2, 0 },
                    new double[] { 0, -Math.PI / 2, 0 },
                    new double[] { 2, 0, 0 },
                }
            );*/

            /*double MaxStep = 1;
            Manip.q_ranges = new double[Manip.l.Length, 2];
            for (int i = 0; i < Manip.l.Length; i++)
            {
                Manip.q_ranges[i, 0] = -MaxStep;
                Manip.q_ranges[i, 1] = MaxStep;
            }*/

            //obstacles
            Obstacles = new Obstacle[2];
            Obstacles[0] = new Obstacle(Primitives.Sphere(1, new Point(0, 1.5, 0), 2000));
            Obstacles[1] = new Obstacle(Primitives.Sphere(1, new Point(-2.2, 0, 0), 2000));
            States["Obstacles"] = true;
            /*Obstacles = new Obstacle[2];
            Point[] obst_data = new Point[16 * 128];
            for (int i = 0; i < 16; i++)
            {
                Matrix rot = Matrix.Rotation(1, i * 2 * Math.PI / 16);

                for (int j = 0; j < 128; j++)
                {
                    obst_data[128 * i + j] = rot * new Point
                    (
                        Math.Cos(j * Math.PI / 64),
                        Math.Sin(j * Math.PI / 64),
                        0
                    ) + new Vector(0, 1.5, 0);
                }
            }
            Obstacles[0] = new Obstacle(obst_data);
            for (int i = 0; i < 16; i++)
            {
                Matrix rot = Matrix.Rotation(1, i * 2 * Math.PI / 16);

                for (int j = 0; j < 128; j++)
                {
                    obst_data[128 * i + j] = rot * new Point
                    (
                        Math.Cos(j * Math.PI / 64),
                        Math.Sin(j * Math.PI / 64),
                        0
                    ) + new Vector(-2.2, 0, 0);
                }
            }
            Obstacles[1] = new Obstacle(obst_data);
            States["Obstacles"] = true;*/

            //goal
            Goal = new Point(-2.2, 0, -2);
            //Goal = new Point(-2 * Math.Sqrt(2) - 2, 0, 0);
            States["Goal"] = true;

            //initialize genetic algorithm parameters
            Algorithm.Initialize(Manip, Obstacles);

            IKP Solver = new IKP(0.02, Manip.l.Length, 10, 0.2, 50);

            Attractors = new List<Attractor>();

            Random rng = new Random();
            double work_radius = 2 * Manip.l.Sum(), x, y_pos, y, z_pos, z;

            //adding main attractor
            Point AttrPoint = Goal;

            double AttrWeight = Manip.DistanceTo(Goal);

            Point[] AttrArea = new Point[8 * 8];
            double r = 0.05 * Math.Pow(AttrWeight / Manip.DistanceTo(Goal), 4);
            for (int i = 0; i < 8; i++)
            {
                Matrix rot = Matrix.Rotation(1, i * 2 * Math.PI / 8);

                for (int j = 0; j < 8; j++)
                {
                    AttrArea[8 * i + j] = rot * new Point
                    (
                        r * Math.Cos(j * Math.PI / 4),
                        r * Math.Sin(j * Math.PI / 4),
                        0
                    ) + new Vector(AttrPoint);
                }
            }

            Attractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));

            //adding ancillary attractors
            while (Attractors.Count < 5000)
            {
                x = -work_radius + rng.NextDouble() * 2 * work_radius;
                y_pos = Math.Sqrt(work_radius * work_radius - x * x);
                y = -y_pos + rng.NextDouble() * 2 * y_pos;
                z_pos = Math.Sqrt(work_radius * work_radius - x * x - y * y);
                z = -z_pos + rng.NextDouble() * 2 * z_pos;

                work_radius -= 2 * Manip.l.Sum() / 10000;

                Point p = new Point(x, y, z) + Manip.Base;
                bool collision = false;
                foreach (var obst in Obstacles)
                {
                    if (obst.Contains(p))
                    {
                        collision = true;
                        break;
                    }
                }

                if (!collision)
                {
                    AttrPoint = p;

                    AttrWeight = Manip.DistanceTo(p) + Goal.DistanceTo(p);

                    AttrArea = new Point[8 * 8];
                    r = 0.05 * Math.Pow(AttrWeight / Manip.DistanceTo(Goal), 4);
                    for (int i = 0; i < 8; i++)
                    {
                        Matrix rot = Matrix.Rotation(1, i * 2 * Math.PI / 8);

                        for (int j = 0; j < 8; j++)
                        {
                            AttrArea[8 * i + j] = rot * new Point
                            (
                                r * Math.Cos(j * Math.PI / 4),
                                r * Math.Sin(j * Math.PI / 4),
                                0
                            ) + new Vector(AttrPoint);
                        }
                    }

                    Attractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));
                }
            }
            States["Attractors"] = true;

            Generator.Solver = Solver;
            Generator.RRT(new Random(), ref Tree, Goal, Manip, Obstacles, 10000, 0.04);
            //Tree.RectifyWhole();

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
            for (int i = 0; i < configs.Count; i++)
            {                
                Manip.q = configs[i];
                Joints.Add(Manip.DKP);
            }
            States["Joints"] = true;
        }
    }
}
