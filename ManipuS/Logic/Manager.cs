using System;
using System.Collections.Generic;
using System.Numerics;
using BulletSharp.Math;
using Logic.InverseKinematics;
using Logic.PathPlanning;

using Vector3 = System.Numerics.Vector3;

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
            var IB = WorkspaceBuffer.InverseKinematicsBuffer;
            var PB = WorkspaceBuffer.PathPlanningBuffer;

            // obstacles
            Obstacles = new Obstacle[OB.Length];
            for (int i = 0; i < OB.Length; i++)
            {
                //Obstacles[i] = new Obstacle(Primitives.Cube(0.5f, 0.5f, 0.5f, new Graphics.MeshMaterial
                //{
                //    Ambient = new OpenTK.Vector4(0.1f, 0.1f, 0.0f, 1.0f),
                //    Diffuse = new OpenTK.Vector4(0.8f, 0.8f, 0.0f, 1.0f),
                //    Specular = new OpenTK.Vector4(0.5f, 0.5f, 0.0f, 1.0f),
                //    Shininess = 8
                //}), new BoxCollider(0.5f, 0.5f, 0.5f), new ImpDualQuat(OB[i].Center));

                //Obstacles[i] = new Obstacle(Primitives.Sphere(0.5f, 50, 25, new Graphics.MeshMaterial
                //{
                //    Ambient = new OpenTK.Vector4(0.1f, 0.1f, 0.0f, 1.0f),
                //    Diffuse = new OpenTK.Vector4(0.8f, 0.8f, 0.0f, 1.0f),
                //    Specular = new OpenTK.Vector4(0.5f, 0.5f, 0.0f, 1.0f),
                //    Shininess = 8
                //}), new SphereCollider(0.5f, 50, 25), new ImpDualQuat(OB[i].Center));

                //Obstacles[i] = new Obstacle(Primitives.Cylinder(0.25f, 1, 1, 50, new Graphics.MeshMaterial
                //{
                //    Ambient = new OpenTK.Vector4(0.1f, 0.1f, 0.0f, 1.0f),
                //    Diffuse = new OpenTK.Vector4(0.8f, 0.8f, 0.0f, 1.0f),
                //    Specular = new OpenTK.Vector4(0.5f, 0.5f, 0.0f, 1.0f),
                //    Shininess = 8
                //}), CylinderCollider.CreateKinematic(0.25f, 1, 1, 50), Matrix.Translation(OB[i].Center.X, OB[i].Center.Y, OB[i].Center.Z));

                //Obstacles[i] = new Obstacle(Primitives.SpherePointCloud(OB[i].Radius, Vector3.Zero, OB[i].PointsNum), new ImpDualQuat(OB[i].Center), ColliderShape.Sphere);
            }

            Manipulators = new Manipulator[MB.Length];
            for (int i = 0; i < MB.Length; i++)
            {
                Manipulators[i] = new Manipulator(MB[i]);

                IKSolver solver = default;
                switch (IB.InverseKinematicsSolverID)
                {
                    case 0:
                        solver = new Jacobian(IB.Precision, IB.StepSize, IB.MaxTime);
                        break;
                    case 1:
                        solver = new HillClimbing(IB.Precision, IB.StepSize, IB.MaxTime);
                        break;
                }

                PathPlanner planner = default;
                switch (PB.PathPlannerID)
                {
                    case 0:
                        planner = new DynamicRRT(PB.k, false, PB.d, PB.k / 10);
                        break;
                    case 1:
                        throw new NotImplementedException("The Genetic algorithm planner is not yet implemented!");
                        break;
                }

                Manipulators[i].Controller = new MotionController(Obstacles, Manipulators[i], planner, solver, 
                    new Jacobian(IB.Precision, IB.StepSize, IB.MaxTime), 2 * PB.d);

                var manip = Manipulators[i];
                manip.Attractors = new List<Attractor>();

                double work_radius = manip.WorkspaceRadius, x, yPos, y, zPos, z;

                // adding main attractor
                Vector3 attrPoint = manip.Goal;
                float attrWeight = manip.DistanceTo(manip.Goal);
                float attrRadius = PB.d * (float)Math.Pow(attrWeight / manip.DistanceTo(manip.Goal), 4);

                manip.Attractors.Add(new Attractor(attrPoint, attrWeight, attrRadius));

                // adding ancillary attractors
                while (manip.Attractors.Count < PB.AttrNum)
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
                        attrRadius = PB.d * (float)Math.Pow(attrWeight / manip.DistanceTo(manip.Goal), 4);

                        manip.Attractors.Add(new Attractor(attrPoint, attrWeight, attrRadius));
                    }
                }
            }            
        }

        public static void Plan(Manipulator manip)
        {
            //var AD = WorkspaceBuffer.AlgBuffer;

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
