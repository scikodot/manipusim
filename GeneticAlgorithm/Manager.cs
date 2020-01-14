using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;

namespace Logic
{
    class Manager
    {
        public static Manipulator[] Manipulators;
        public static Obstacle[] Obstacles;

        public static void Initialize()
        {
            //manipulators
            Manipulators = new Manipulator[]
            {
                new Manipulator
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
                    new TupleDH[]
                    {
                        new TupleDH((m) => { return m.q[0]; }, 0.2, Math.PI / 2, 0),
                        new TupleDH((m) => { return m.q[1] - Math.PI / 2; }, 0, 0, 2),
                        new TupleDH((m) => { return m.q[2]; }, 0, 0, 2),
                        new TupleDH((m) => { return m.q[3]; }, 0, 0, 2)
                    },
                    new Point(-2.2, 0, -2)
                ),
                new Manipulator
                (
                    new Point(2, 0, -2),
                    new double[] { 0.2, 2, 2, 2 },
                    Misc.ToRad(new double[] { 0, 0, 0, 0 }),
                    new double[,]
                    {
                        { -180, 180 },
                        { -180, 180 },
                        { -180, 180 },
                        { -180, 180 }
                    },
                    new TupleDH[]
                    {
                        new TupleDH((m) => { return m.q[0]; }, 0.2, Math.PI / 2, 0),
                        new TupleDH((m) => { return m.q[1] - Math.PI / 2; }, 0, 0, 2),
                        new TupleDH((m) => { return m.q[2]; }, 0, 0, 2),
                        new TupleDH((m) => { return m.q[3]; }, 0, 0, 2)
                    },
                    new Point(2.2, 0, 2)
                )
            };

            //obstacles
            Obstacles = new Obstacle[2]
            {
                new Obstacle(Primitives.Sphere(1, new Point(0, 1.5, 0), 2000)),
                new Obstacle(Primitives.Sphere(1, new Point(-2.2, 0, 0), 2000))
            };

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

            //initialize algorithm parameters
            Algorithm.Initialize(Obstacles);
        }

        public static void Execute(Manipulator manip)
        {
            manip.States = new Dictionary<string, bool>
            {
                { "Goal", false },
                { "Path", false },
                { "Joints", false },
                { "Tree", false },
                { "Attractors", false }
            };

            manip.States["Goal"] = true;

            manip.Attractors = new List<Attractor>();

            Random rng = new Random();
            double work_radius = 2 * manip.l.Sum(), x, y_pos, y, z_pos, z;

            //adding main attractor
            Point AttrPoint = manip.Goal;

            double AttrWeight = manip.DistanceTo(manip.Goal);

            double r = 0.05 * Math.Pow(AttrWeight / manip.DistanceTo(manip.Goal), 4);
            Point[] AttrArea = Primitives.Sphere(r, AttrPoint, 64);  //new Point[8 * 8];
            /*double r = 0.05 * Math.Pow(AttrWeight / Manip.DistanceTo(Goal), 4);
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
            }*/

            manip.Attractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));

            //adding ancillary attractors
            while (manip.Attractors.Count < 5000)
            {
                x = -work_radius + rng.NextDouble() * 2 * work_radius;
                y_pos = Math.Sqrt(work_radius * work_radius - x * x);
                y = -y_pos + rng.NextDouble() * 2 * y_pos;
                z_pos = Math.Sqrt(work_radius * work_radius - x * x - y * y);
                z = -z_pos + rng.NextDouble() * 2 * z_pos;

                work_radius -= 2 * manip.l.Sum() / 10000;

                Point p = new Point(x, y, z) + manip.Base;
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

                    AttrWeight = manip.DistanceTo(p) + manip.Goal.DistanceTo(p);

                    r = 0.05 * Math.Pow(AttrWeight / manip.DistanceTo(manip.Goal), 4);
                    AttrArea = Primitives.Sphere(r, AttrPoint, 64);  //new Point[8 * 8];
                    /*r = 0.05 * Math.Pow(AttrWeight / Manip.DistanceTo(Goal), 4);
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
                    }*/

                    manip.Attractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));
                }
            }
            manip.States["Attractors"] = true;
            
            PathPlanner.RRT(rng, manip, Obstacles, new IKP(0.02, manip.q.Length, 10, 0.2, 50), 10000, 0.04);
            //Tree.RectifyWhole();

            Tree.Node start = manip.Tree.Min(manip.Goal), node_curr = start;
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
            manip.States["Tree"] = true;

            manip.Path = path;
            manip.States["Path"] = true;
            
            for (int i = 0; i < configs.Count; i++)
            {                
                manip.q = configs[i];
                manip.Joints.Add(manip.DKP);
            }
            manip.States["Joints"] = true;
        }
    }
}
