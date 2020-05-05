using System;
using System.Collections.Generic;
using System.Numerics;
using BulletSharp;
using BulletSharp.Math;
using Logic.InverseKinematics;
using Logic.PathPlanning;
using Physics;
using Vector3 = System.Numerics.Vector3;

namespace Logic
{
    public static class ManipHandler
    {
        public static List<Manipulator> Manipulators { get; } = new List<Manipulator>();

        public static int Count => Manipulators.Count;

        public static void Add(Manipulator manipulator)
        {
            Manipulators.Add(manipulator);

            var IB = WorkspaceBuffer.InverseKinematicsBuffer;
            var PB = WorkspaceBuffer.PathPlanningBuffer;

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

            manipulator.Controller = new MotionController(ObstacleHandler.Obstacles.ToArray(), manipulator, planner, solver,
                   new Jacobian(IB.Precision, IB.StepSize, IB.MaxTime), 2 * PB.d);
        }

        public static void Remove(Manipulator manipulator)
        {
            // TODO: implement
        }

        //public static void Initialize()
        //{
        //    Obstacles[i] = new Obstacle(Primitives.Cube(0.5f, 0.5f, 0.5f, new Graphics.MeshMaterial
        //    {
        //        Ambient = new OpenTK.Vector4(0.1f, 0.1f, 0.0f, 1.0f),
        //        Diffuse = new OpenTK.Vector4(0.8f, 0.8f, 0.0f, 1.0f),
        //        Specular = new OpenTK.Vector4(0.5f, 0.5f, 0.0f, 1.0f),
        //        Shininess = 8
        //    }), new BoxCollider(0.5f, 0.5f, 0.5f), new ImpDualQuat(OB[i].Center));

        //    Obstacles[i] = new Obstacle(Primitives.Sphere(0.5f, 50, 25, new Graphics.MeshMaterial
        //    {
        //        Ambient = new OpenTK.Vector4(0.1f, 0.1f, 0.0f, 1.0f),
        //        Diffuse = new OpenTK.Vector4(0.8f, 0.8f, 0.0f, 1.0f),
        //        Specular = new OpenTK.Vector4(0.5f, 0.5f, 0.0f, 1.0f),
        //        Shininess = 8
        //    }), new SphereCollider(0.5f, 50, 25), new ImpDualQuat(OB[i].Center));

        //    Obstacles[i] = new Obstacle(Primitives.Cylinder(0.25f, 1, 1, 50, new Graphics.MeshMaterial
        //    {
        //        Ambient = new OpenTK.Vector4(0.1f, 0.1f, 0.0f, 1.0f),
        //        Diffuse = new OpenTK.Vector4(0.8f, 0.8f, 0.0f, 1.0f),
        //        Specular = new OpenTK.Vector4(0.5f, 0.5f, 0.0f, 1.0f),
        //        Shininess = 8
        //    }), PhysicsHandler.CreateKinematicCollider(new CylinderShape(0.25f, 1, 0.25f), Matrix.Translation(OB[i].Center.X, OB[i].Center.Y, OB[i].Center.Z)));

        //    Obstacles[i] = new Obstacle(Primitives.SpherePointCloud(OB[i].Radius, Vector3.Zero, OB[i].PointsNum), new ImpDualQuat(OB[i].Center), ColliderShape.Sphere);
        //}

        public static void RunControl()
        {
            foreach (var manip in Manipulators)
            {
                manip.Controller.Run();
            }
        }

        public static void AbortControl()
        {
            foreach (var manip in Manipulators)
            {
                manip.Controller.Abort();
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
