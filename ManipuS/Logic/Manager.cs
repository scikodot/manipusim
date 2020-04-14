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
            Random rng = new Random();

            var MB = WorkspaceBuffer.ManipBuffer;
            var OB = WorkspaceBuffer.ObstBuffer;
            var AB = WorkspaceBuffer.AlgBuffer;

            // obstacles
            Obstacles = new Obstacle[OB.Length];
            for (int i = 0; i < OB.Length; i++)
            {
                Obstacles[i] = new Obstacle(Primitives.Sphere(OB[i].Radius, Vector3.Zero, OB[i].PointsNum, new Random()), new ImpDualQuat(OB[i].Center), ColliderShape.Sphere);
            }

            // manipulators
            //Manipulators = new Manipulator[1];
            //Manipulators[0] = new Manipulator(LD, JD, new TupleDH[]
            //    {
            //        new TupleDH(0, JD[0].Length / 2 + JD[1].Length / 2 + LD[0].Length, 90 * (float)Math.PI / 180, 0),
            //        new TupleDH(-90 * (float)Math.PI / 180, 0, 0, JD[1].Length / 2 + JD[2].Length / 2 + LD[1].Length),
            //        new TupleDH(0, 0, 0, JD[2].Length / 2 + 0.1f + LD[2].Length),
            //        new TupleDH(90 * (float)Math.PI / 180, 0, -90 * (float)Math.PI / 180, 0)
            //    });
            //Manipulators[0].Goal = new Vector3(0, 0.5f, 2.5f);
            //Manipulators[0].controller = new MotionController(Obstacles, Manipulators[0],
            //    new DynamicRRT(AD.k, true, AD.d, AD.k / 100),
            //    new Jacobian(AD.Precision, AD.StepSize, AD.MaxTime));

            Manipulators = new Manipulator[MB.Length];
            for (int i = 0; i < MB.Length; i++)
            {
                Manipulators[i] = new Manipulator(MB[i]);

                IKSolver solver = default;
                switch (AB.InverseKinematicsSolverID)
                {
                    case 0:
                        solver = new Jacobian(AB.Precision, AB.StepSize, AB.MaxTime);
                        break;
                    case 1:
                        solver = new HillClimbing(AB.Precision, AB.StepSize, AB.MaxTime);
                        break;
                }

                PathPlanner planner = default;
                switch (AB.PathPlannerID)
                {
                    case 0:
                        planner = new DynamicRRT(AB.k, false, AB.d, AB.k / 100);
                        break;
                    case 1:
                        throw new NotImplementedException("The Genetic algorithm planner is not yet implemented!");
                        break;
                }

                Manipulators[i].Controller = new MotionController(Obstacles, Manipulators[i], planner, solver, 
                    new Jacobian(AB.Precision, AB.StepSize, AB.MaxTime), 2 * AB.d);

                var manip = Manipulators[i];
                manip.Attractors = new List<Attractor>();

                double work_radius = manip.WorkspaceRadius, x, yPos, y, zPos, z;

                // adding main attractor
                Vector3 attrPoint = manip.Goal;
                float attrWeight = manip.DistanceTo(manip.Goal);
                float attrRadius = WorkspaceBuffer.AlgBuffer.d * (float)Math.Pow(attrWeight / manip.DistanceTo(manip.Goal), 4);
                Vector3[] attrArea = Primitives.Sphere(attrRadius, attrPoint, 32, new Random());

                manip.Attractors.Add(new Attractor(attrPoint, attrWeight, attrArea, attrRadius));
                manip.States["Goal"] = true;

                // adding ancillary attractors
                while (manip.Attractors.Count < AB.AttrNum)
                {
                    // generating attractor point
                    x = work_radius * (2 * rng.NextDouble() - 1);
                    yPos = Math.Sqrt(work_radius * work_radius - x * x);
                    y = yPos * (2 * rng.NextDouble() - 1);
                    zPos = Math.Sqrt(yPos * yPos - y * y);
                    z = zPos * (2 * rng.NextDouble() - 1);

                    Vector3 point = new Vector3((float)x, (float)y, (float)z) + manip.Base;

                    // checking whether the attractor is inside any obstacle or not
                    bool collision = false;
                    foreach (var obst in Obstacles)
                    {
                        if (obst.Contains(point))
                        {
                            collision = true;
                            break;
                        }
                    }

                    if (!collision)  // TODO: consider creating a list of bad attractors; they may serve as repulsion points
                    {
                        // adding attractor to the list
                        attrPoint = point;
                        attrWeight = manip.DistanceTo(point) + manip.Goal.DistanceTo(point);
                        attrRadius = AB.d * (float)Math.Pow(attrWeight / manip.DistanceTo(manip.Goal), 4);
                        attrArea = Primitives.Sphere(attrRadius, attrPoint, 32, new Random());

                        manip.Attractors.Add(new Attractor(attrPoint, attrWeight, attrArea, attrRadius));
                    }
                }
                manip.States["Attractors"] = true;
            }            
        }

        public static void Plan(Manipulator manip)
        {
            var AD = WorkspaceBuffer.AlgBuffer;

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

            /*var solver = new Jacobian(Obstacles, AD.Precision, AD.StepSize, AD.MaxTime);  // TODO: solvers should be declared inside planners!
            var planner = new DynamicRRT(Obstacles, solver, AD.k, true, AD.d, AD.k / 100);
            planner.Start(manip, Vector3.Zero);*/

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
