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

            var manip = Manipulators[0];
            manip.GoodAttractors = new List<Attractor>();
            manip.BadAttractors = new List<Attractor>();

            Random rng = new Random();
            double work_radius = manip.WorkspaceRadius, x, y_pos, y, z_pos, z;

            // adding main attractor
            Point AttrPoint = manip.Goal;

            double AttrWeight = manip.DistanceTo(manip.Goal);

            double r = Dispatcher.WorkspaceBuffer.AlgBuffer.d * Math.Pow(AttrWeight / manip.DistanceTo(manip.Goal), 4);
            Point[] AttrArea = Primitives.Sphere(r, AttrPoint, 64, new Random());

            manip.GoodAttractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));
            manip.States["Goal"] = true;

            var AD = Dispatcher.WorkspaceBuffer.AlgBuffer;

            // adding ancillary attractors
            while (manip.GoodAttractors.Count < AD.AttrNum)
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

                // adding attractor to the list
                AttrPoint = p;

                AttrWeight = manip.DistanceTo(p) + manip.Goal.DistanceTo(p);

                r = AD.d * Math.Pow(AttrWeight / manip.DistanceTo(manip.Goal), 4);
                AttrArea = Primitives.Sphere(r, AttrPoint, 64, new Random());

                if (!collision)
                {
                    manip.GoodAttractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));
                }
                else
                {
                    manip.BadAttractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));
                }
            }
            manip.States["Attractors"] = true;
        }

        public static void Execute(Manipulator manip)
        {
            var AD = Dispatcher.WorkspaceBuffer.AlgBuffer;

            // generating random tree
            var solver = new HillClimbing(Obstacles, manip.q.Length, AD.Precision, AD.StepSize, AD.MaxTime);
            var resRRT = PathPlanner.RRT(manip, Obstacles, solver, AD.k, AD.d, false);

            /*var resGA = PathPlanner.GeneticAlgorithm(manip, Obstacles, manip.Goal, resRRT.Item2.ToArray(), 
                0.99, manip.Joints.Length, 20, 0.95, 0.1, 10000, 
                PathPlanner.OptimizationCriterion.CollisionFree, 
                PathPlanner.SelectionMode.NormalDistribution, 
                PathPlanner.CrossoverMode.WeightedMean, 
                t => t * Math.PI / 180);*/

            var input = new (Point, double[])[resRRT.Item1.Count];
            for (int i = 0; i < input.Length; i++)
            {
                input[i].Item1 = resRRT.Item1[i];
                input[i].Item2 = resRRT.Item2[i];
            }

            var jac = new Jacobian(Obstacles, manip.q.Length, AD.Precision, AD.StepSize, AD.MaxTime);
            var resGA = PathPlanner.GeneticAlgorithmD(manip, Obstacles, manip.Goal, jac, 
                input, 0.99, manip.Joints.Length, 4, 0.95, 0.1, 10000,
                PathPlanner.OptimizationCriterion.CollisionFree,
                PathPlanner.SelectionMode.NormalDistribution,
                PathPlanner.CrossoverMode.WeightedMean,
                t => t * Math.PI / 180);

            // acquiring all the points and configurations along the path
            manip.Path = resGA.Item1;
            manip.States["Path"] = true;
            manip.Configs = resGA.Item2;
        }
    }
}
