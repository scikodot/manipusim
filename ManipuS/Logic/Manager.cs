using System;
using System.Collections.Generic;
using System.Linq;
using Logic.InverseKinematics;
using Logic.PathPlanning;

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
                    new TupleDH(0, JD[0].Length + LD[0].Length, 90 * (float)Math.PI / 180, 0),
                    new TupleDH(-90 * (float)Math.PI / 180, 0, 0, JD[1].Length + LD[1].Length),
                    new TupleDH(0, 0, 0, JD[2].Length + LD[2].Length)
                });
            Manipulators[0].Goal = new Vector3(0, 0.5f, 2.5f);

            // obstacles
            Obstacles = new Obstacle[OD.Length];
            for (int i = 0; i < OD.Length; i++)
            {
                Obstacles[i] = new Obstacle(Primitives.Sphere(OD[i].r, new Vector3(OD[i].c.X, OD[i].c.Y, OD[i].c.Z), OD[i].Vector3s_num, new Random()), ColliderShape.Sphere);
            }

            var manip = Manipulators[0];
            manip.GoodAttractors = new List<Attractor>();
            manip.BadAttractors = new List<Attractor>();

            Random rng = new Random();
            float work_radius = manip.WorkspaceRadius, x, y_pos, y, z_pos, z;

            // adding main attractor
            Vector3 AttrVector3 = manip.Goal;

            float AttrWeight = manip.DistanceTo(manip.Goal);

            float r = Dispatcher.WorkspaceBuffer.AlgBuffer.d * (float)Math.Pow(AttrWeight / manip.DistanceTo(manip.Goal), 4);
            Vector3[] AttrArea = Primitives.Sphere(r, AttrVector3, 64, new Random());

            manip.GoodAttractors.Add(new Attractor(AttrVector3, AttrWeight, AttrArea, r));
            manip.States["Goal"] = true;

            var AD = Dispatcher.WorkspaceBuffer.AlgBuffer;

            // adding ancillary attractors
            while (manip.GoodAttractors.Count < AD.AttrNum)
            {
                // generating attractor Vector3
                x = -work_radius + (float)rng.NextDouble() * 2 * work_radius;
                y_pos = (float)Math.Sqrt(work_radius * work_radius - x * x);
                y = -y_pos + (float)rng.NextDouble() * 2 * y_pos;
                z_pos = (float)Math.Sqrt(work_radius * work_radius - x * x - y * y);
                z = -z_pos + (float)rng.NextDouble() * 2 * z_pos;

                Vector3 p = new Vector3(x, y, z) + manip.Base;

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
                AttrVector3 = p;

                AttrWeight = manip.DistanceTo(p) + manip.Goal.DistanceTo(p);

                r = AD.d * (float)Math.Pow(AttrWeight / manip.DistanceTo(manip.Goal), 4);
                AttrArea = Primitives.Sphere(r, AttrVector3, 64, new Random());

                if (!collision)
                {
                    manip.GoodAttractors.Add(new Attractor(AttrVector3, AttrWeight, AttrArea, r));
                }
                else
                {
                    manip.BadAttractors.Add(new Attractor(AttrVector3, AttrWeight, AttrArea, r));
                }
            }
            manip.States["Attractors"] = true;
        }

        public static void Execute(Manipulator manip)
        {
            var AD = Dispatcher.WorkspaceBuffer.AlgBuffer;

            /*// generating random tree
            var solver = new HillClimbing(Obstacles, AD.Precision, AD.StepSize, AD.MaxTime);  // TODO: solvers should be declared inside planners!
            var planner = new RRT(Obstacles, solver, AD.k, true, AD.d);
            PathPlanner pp = planner;
            var resRRT = pp.Execute(manip, null);

            // acquiring all the Vector3s and configurations along the path
            manip.Path = resRRT.Item1;
            manip.States["Path"] = true;
            manip.Configs = resRRT.Item2;*/

            /*// generating random tree
            var solver = new HillClimbing(Obstacles, AD.Precision, AD.StepSize, AD.MaxTime);  // TODO: solvers should be declared inside planners!
            var planner = new DynamicRRT(Obstacles, solver, AD.k, true, AD.d, AD.k / 100);
            PathPlanner pp = planner;
            var resRRT = pp.Execute(manip, null);

            // acquiring all the Vector3s and configurations along the path
            manip.Path = resRRT.Item1;
            manip.States["Path"] = true;
            manip.Configs = resRRT.Item2;*/

            var solver = new Jacobian(Obstacles, AD.Precision, AD.StepSize, AD.MaxTime);  // TODO: solvers should be declared inside planners!
            var planner = new DynamicRRT(Obstacles, solver, AD.k, true, AD.d, AD.k / 100);
            planner.Start(manip, Vector3.Zero);

            /*var resGA = PathPlanner.GeneticAlgorithm(manip, Obstacles, manip.Goal, resRRT.Item2.ToArray(), 
                0.99, manip.Joints.Length, 20, 0.95, 0.1, 10000, 
                PathPlanner.OptimizationCriterion.CollisionFree, 
                PathPlanner.SelectionMode.NormalDistribution, 
                PathPlanner.CrossoverMode.WeightedMean, 
                t => t * Math.PI / 180);*/

            /*var input = new (Vector3, float[])[resRRT.Item1.Count];
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

            // acquiring all the Vector3s and configurations along the path
            manip.Path = resGA.Item1;
            manip.States["Path"] = true;
            manip.Configs = resGA.Item2;*/
        }
    }
}
