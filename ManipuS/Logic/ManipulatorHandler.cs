using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using BulletSharp;
using BulletSharp.Math;
using Graphics;
using Logic.InverseKinematics;
using Logic.PathPlanning;
using Physics;
using Vector3 = System.Numerics.Vector3;

namespace Logic
{
    public static class ManipulatorHandler
    {
        private static int _defaultLinksNumber = 3;
        public static ref int DefaultLinksNumber => ref _defaultLinksNumber;

        private static Model _defaultJointModel;
        private static Model _defaultLinkModel;
        private static Model _defaultGripperModel;

        private static JointData _defaultJoint = new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new Vector2(-180, 180)
        };
        private static LinkData _defaultLink = new LinkData
        {
            Length = 1
        };
        private static JointData _defaultGripper = new JointData
        {
            Length = 0.2f,
            q = 0,
            qRanges = new Vector2(-180, 180)
        };
        private static Vector3 _defaultGoal = new Vector3(0.0f/*-1.0f*/, 0.5f, -2.0f);

        private static InverseKinematicsData _defaultInverseKinematicsSolver = new InverseKinematicsData
        {
            Precision = 0.02f,
            StepSize = 2,
            MaxTime = 4
        };
        private static PathPlanningData _defaultPathPlanner = new PathPlanningData
        {
            AttrNum = 5000,
            k = 10000,
            d = 0.04f
        };

        public static List<Manipulator> Manipulators { get; } = new List<Manipulator>();

        public static int Count => Manipulators.Count;

        public static void LoadDefaultModels()
        {
            Dispatcher.ActiveTasks.Add(Task.Run(() =>
            {
                // load components' models
                _defaultJointModel = new Model(InputHandler.JointPath);
                _defaultLinkModel = new Model(InputHandler.LinkPath);
                _defaultGripperModel = new Model(InputHandler.GripperPath);
            }));
        }

        public static Manipulator CreateDefaultManipulator(int linksNumber)
        {
            // set links' parameters
            var links = new LinkData[linksNumber];
            links.Fill(_defaultLink);

            // define model and collider for each link
            for (int i = 0; i < links.Length; i++)
            {
                links[i].Model = _defaultLinkModel.ShallowCopy();
                links[i].Collider = PhysicsHandler.CreateKinematicCollider(new CylinderShape(0.15f, 0.5f, 0.15f));
            }

            // set joints' parameters
            var joints = new JointData[linksNumber + 1];
            joints.Fill(_defaultJoint);
            joints[joints.Length - 1] = _defaultGripper;

            // define model and collider for each joint
            for (int i = 0; i < joints.Length - 1; i++)
            {
                joints[i].Model = _defaultJointModel.ShallowCopy();
                joints[i].Collider = PhysicsHandler.CreateKinematicCollider(new SphereShape(0.2f));
            }

            // TODO: gripper collider is not affected by the initial transform; fix!
            joints[linksNumber].Model = _defaultGripperModel.ShallowCopy();
            joints[linksNumber].Collider = PhysicsHandler.CreateKinematicCollider(new SphereShape(0.1f));

            // set joints' axes
            var jointAxes = new Vector3[linksNumber + 1];
            jointAxes[0] = jointAxes[jointAxes.Length - 1] = Vector3.UnitY;
            for (int i = 1; i < linksNumber; i++)
            {
                jointAxes[i] = /*Vector3.UnitX;*/  i % 2 == 0 ? Vector3.UnitZ : Vector3.UnitX;
            }

            // set joints' positions
            var jointPositions = new Vector3[linksNumber + 1];
            jointPositions[0] = Vector3.Zero;
            for (int i = 1; i < linksNumber + 1; i++)
            {
                jointPositions[i] = jointPositions[i - 1] + ((joints[i - 1].Length + joints[i].Length) / 2 + links[i - 1].Length) * Vector3.UnitY;
            }

            // create a default manipulator
            var manipulator = new Manipulator(new ManipData
            {
                N = linksNumber,
                Links = links,
                Joints = joints,
                JointAxes = jointAxes,
                JointPositions = jointPositions,
                Goal = _defaultGoal,
                ShowTree = true
            });

            // add default motion controller default solver and planner
            var defaultSolver = _defaultInverseKinematicsSolver;
            var defaultPlanner = _defaultPathPlanner;

            var solver = new JacobianTranspose(defaultSolver.Precision, defaultSolver.StepSize, defaultSolver.MaxTime);
            var planner = /*new GeneticAlgorithm(defaultPlanner.k, true, 10, 0.95f, 0.1f);  */new DynamicRRT(defaultPlanner.k, true, defaultPlanner.d, defaultPlanner.k / 10);
            manipulator.Controller = new MotionController(ObstacleHandler.Obstacles.ToArray(), manipulator, planner, solver,
                   new DampedLeastSquares(defaultSolver.Precision, defaultSolver.StepSize, defaultSolver.MaxTime), 2 * defaultPlanner.d);

            // add manipulator to the list
            Add(manipulator);

            return manipulator;
        }

        public static void Add(Manipulator manipulator)
        {
            Manipulators.Add(manipulator);
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

        public static void ToDesign()
        {
            // stop threads
            AbortControl();

            // reset positions
            Reset();
        }

        public static void ToAnimate()
        {
            // start threads
            RunControl();
        }

        public static void UpdateDesign()
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.UpdateStateDesign();
            }
        }

        public static void UpdateModel()
        {
            foreach(var manipulator in Manipulators)
            {
                manipulator.UpdateModel();
            }
        }

        public static void RenderUnselected(Shader shader)
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.RenderUnselected(shader);
            }
        }

        public static void RenderSelected(Shader shader)
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.RenderSelected(shader);
            }
        }

        public static void Reset()
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.Reset();
            }
        }

        public static void RunControl()
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.Controller.Run();
            }
        }

        public static void AbortControl()
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.Controller.Abort();
            }
        }

        public static void Dispose()
        {
            // dispose of all the manipulators
            foreach (var manipulator in Manipulators)
            {
                manipulator.Dispose();
            }
        }

        public static void Plan(Manipulator manip)
        {
            /*var resGA = PathPlanner.GeneticAlgorithm(manip, Obstacles, manip.Goal, resRRT.Item2.ToArray(), 
                0.99, manip.Joints.Length, 20, 0.95, 0.1, 10000, 
                PathPlanner.OptimizationCriterion.CollisionFree, 
                PathPlanner.SelectionMode.NormalDistribution, 
                PathPlanner.CrossoverMode.WeightedMean, 
                t => t * Math.PI / 180);*/

            /*var jac = new Jacobian(Obstacles, manip.q.Length, AD.Precision, AD.StepSize, AD.MaxTime);
            var resGA = PathPlanner.GeneticAlgorithmD(manip, Obstacles, manip.Goal, jac, 
                input, 0.99, manip.Joints.Length, 4, 0.95, 0.1, 10000,
                PathPlanner.OptimizationCriterion.CollisionFree,
                PathPlanner.SelectionMode.NormalDistribution,
                PathPlanner.CrossoverMode.WeightedMean,
                t => t * Math.PI / 180);*/
        }
    }
}
