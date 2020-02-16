using System;
using System.Collections.Generic;
using System.Linq;

namespace Logic
{
    class Manager
    {
        public static Manipulator[] Manipulators;
        public static Obstacle[] Obstacles;

        public static void Initialize()
        {
            var LD = Dispatcher.WorkspaceBuffer.LinkBuffer;
            var JD = Dispatcher.WorkspaceBuffer.JointBuffer;
            var OD = Dispatcher.WorkspaceBuffer.ObstBuffer;

            // manipulators
            Manipulators = new Manipulator[1];
            Manipulators[0] = new Manipulator(LD, JD, new TupleDH[]
                {
                    new TupleDH((j) => { return j.q; }, JD[0].Length + LD[0].Length, 90 * Math.PI / 180, 0),
                    new TupleDH((j) => { return j.q - 90 * Math.PI / 180; }, 0, 0, JD[1].Length + LD[1].Length),
                    new TupleDH((j) => { return j.q; }, 0, 0, JD[2].Length + LD[2].Length)
                });
            Manipulators[0].Goal = new Point(0, 0.5, 2.5);

            // obstacles
            Obstacles = new Obstacle[OD.Length];
            for (int i = 0; i < OD.Length; i++)
            {
                Obstacles[i] = new Obstacle(Primitives.Sphere(OD[i].r, new Point(OD[i].c.X, OD[i].c.Y, OD[i].c.Z), OD[i].points_num, new Random()), ColliderShape.Sphere);
            }
        }

        public static void Execute(Manipulator manip)
        {
            manip.Attractors = new List<Attractor>();

            Random rng = new Random();
            double work_radius = manip.WorkspaceRadius, x, y_pos, y, z_pos, z;

            // adding main attractor
            Point AttrPoint = manip.Goal;

            double AttrWeight = manip.DistanceTo(manip.Goal);

            double r = Dispatcher.WorkspaceBuffer.AlgBuffer.d * Math.Pow(AttrWeight / manip.DistanceTo(manip.Goal), 4);
            Point[] AttrArea = Primitives.Sphere(r, AttrPoint, 64, new Random());

            manip.Attractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));
            manip.States["Goal"] = true;

            var AD = Dispatcher.WorkspaceBuffer.AlgBuffer;

            // adding ancillary attractors
            while (manip.Attractors.Count < AD.AttrNum)
            {
                // generating attractor point
                x = -work_radius + rng.NextDouble() * 2 * work_radius;
                y_pos = Math.Sqrt(work_radius * work_radius - x * x);
                y = -y_pos + rng.NextDouble() * 2 * y_pos;
                z_pos = Math.Sqrt(work_radius * work_radius - x * x - y * y);
                z = -z_pos + rng.NextDouble() * 2 * z_pos;

                Point p = new Point(x, y, z) + manip.Base;

                // checking whether the attractor is inside any obstacle or not
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
                    // adding attractor to the list
                    AttrPoint = p;

                    AttrWeight = manip.DistanceTo(p) + manip.Goal.DistanceTo(p);

                    r = AD.d * Math.Pow(AttrWeight / manip.DistanceTo(manip.Goal), 4);
                    AttrArea = Primitives.Sphere(r, AttrPoint, 64, new Random());

                    manip.Attractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));
                }
            }
            manip.States["Attractors"] = true;

            // generating random tree
            PathPlanner.RRT(rng, manip, Obstacles, new HillClimbing(Obstacles, manip.q.Length, AD.Precision, AD.StepSize, AD.MaxTime), AD.k, AD.d);

            // retrieving resultant path along with respective configurations
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

            // reverting path so that it goes from root to goal
            path.Reverse();
            configs.Reverse();

            manip.Path = path;
            manip.States["Path"] = true;

            // acquiring all the configurations along the path
            manip.Configs = new List<double[]>(configs);
        }
    }
}
