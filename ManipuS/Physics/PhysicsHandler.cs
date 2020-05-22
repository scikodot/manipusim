using BulletSharp;
using BulletSharp.Math;
using Logic;
using System;
using System.Collections.Generic;
using System.Linq;
using Vector3 = BulletSharp.Math.Vector3;

namespace Physics
{
    public enum RigidBodyType
    {
        Static,
        Kinematic,
        Dynamic
    }

    public static class PhysicsHandler
    {
        public static string[] RigidBodyTypes { get; } = Enum.GetNames(typeof(RigidBodyType));

        public static DiscreteDynamicsWorld World { get; }

        private static CollisionDispatcher _dispatcher;
        private static DbvtBroadphase _broadphase;
        private static List<CollisionShape> _collisionShapes = new List<CollisionShape>();
        private static CollisionConfiguration _collisionConf;

        private const int MaxThreadCount = 64;
        private static ConstraintSolverPoolMultiThreaded _solverPool;
        private static SequentialImpulseConstraintSolverMultiThreaded _parallelSolver;
        private static List<TaskScheduler> _schedulers = new List<TaskScheduler>();
        private static int _currentScheduler = 0;

        //public static volatile bool IsUpdating;

        static PhysicsHandler()
        {
            CreateSchedulers();
            NextTaskScheduler();

            using (var collisionConfigurationInfo = new DefaultCollisionConstructionInfo
            {
                DefaultMaxPersistentManifoldPoolSize = 80000,
                DefaultMaxCollisionAlgorithmPoolSize = 80000
            })
            {
                _collisionConf = new DefaultCollisionConfiguration(collisionConfigurationInfo);
            };
            _dispatcher = new CollisionDispatcherMultiThreaded(_collisionConf);
            _broadphase = new DbvtBroadphase();
            _solverPool = new ConstraintSolverPoolMultiThreaded(MaxThreadCount);
            _parallelSolver = new SequentialImpulseConstraintSolverMultiThreaded();
            World = new DiscreteDynamicsWorldMultiThreaded(_dispatcher, _broadphase, _solverPool, _parallelSolver, _collisionConf);
            World.SolverInfo.SolverMode = SolverModes.Simd | SolverModes.UseWarmStarting;

            //// collision configuration contains default setup for memory, collision setup
            //_collisionConf = new DefaultCollisionConfiguration();
            //_dispatcher = new CollisionDispatcherMultiThreaded(_collisionConf);

            //_broadphase = new DbvtBroadphase();
            //World = new DiscreteDynamicsWorldMultiThreaded(_dispatcher, _broadphase, null, null, _collisionConf);

            //World.Gravity = Vector3.Zero;

            //// create the ground
            //var groundShape = new BoxShape(5, 0.125f, 5);
            //_collisionShapes.Add(groundShape);
            //Collider ground = CreateStaticCollider(groundShape, Matrix.Identity);
            //ground.Body.UserObject = "Ground";

            //// create a few dynamic rigidbodies
            //var colShape = new BoxShape(0.5f);
            //_collisionShapes.Add(colShape);

            //float mass = 1.0f;
            //Vector3 localInertia = colShape.CalculateLocalInertia(mass);

            //using (var rbInfo = new RigidBodyConstructionInfo(mass, null, colShape, localInertia))
            //{
            //    // using motionstate is recommended, it provides interpolation capabilities
            //    // and only synchronizes 'active' objects
            //    rbInfo.MotionState = new DefaultMotionState(Matrix.Translation(new Vector3(0.0f, 3.0f, 0.0f)));
            //    var body = new RigidBody(rbInfo);
            //    body.UserIndex = 0;
            //    World.AddRigidBody(body);

            //    rbInfo.MotionState = new DefaultMotionState(Matrix.Translation(new Vector3(0.5f, 4.5f, 0.0f)));
            //    body = new RigidBody(rbInfo);
            //    body.UserIndex = 1;
            //    World.AddRigidBody(body);

            //    rbInfo.MotionState = new DefaultMotionState(Matrix.Translation(new Vector3(1.0f, 6.0f, 0.0f)));
            //    body = new RigidBody(rbInfo);
            //    body.UserIndex = 2;
            //    World.AddRigidBody(body);
            //}
        }

        public static void NextTaskScheduler()
        {
            _currentScheduler++;
            if (_currentScheduler >= _schedulers.Count)
            {
                _currentScheduler = 0;
            }
            TaskScheduler scheduler = _schedulers[_currentScheduler];
            scheduler.NumThreads = scheduler.MaxNumThreads;
            Threads.TaskScheduler = scheduler;
        }

        private static void CreateSchedulers()
        {
            AddScheduler(Threads.GetSequentialTaskScheduler());
            AddScheduler(Threads.GetOpenMPTaskScheduler());
            AddScheduler(Threads.GetTbbTaskScheduler());
            AddScheduler(Threads.GetPplTaskScheduler());
        }

        private static void AddScheduler(TaskScheduler scheduler)
        {
            if (scheduler != null)
            {
                _schedulers.Add(scheduler);
            }
        }

        public static void RayTestRef(ref Vector3 startWorld, ref Vector3 endWorld, ClosestRayResultCallback raycastCallback)
        {
            lock (World)
                World.RayTestRef(ref startWorld, ref endWorld, raycastCallback);
        }

        public static void ContactPairTest(RigidBody body, RigidBody bodyOther, CollisionCallback collisionCallback)
        {
            lock (World)
                World.ContactPairTest(body, bodyOther, collisionCallback);
        }

        public static void ContactTest(RigidBody body, CollisionCallback collisionCallback)
        {
            lock (World)
                World.ContactTest(body, collisionCallback);
        }

        public static void Update(float elapsedTime)
        {
            lock (World)
                World.StepSimulation(elapsedTime);  // TODO: can crash, perhaps due to thread sync absent; fix!!!
        }

        public static void RemoveRigidBody(RigidBody body)
        {
            if (body != null && body.MotionState != null)
            {
                body.MotionState.Dispose();
            }
            World.RemoveRigidBody(body);  // TODO: can crash, perhaps due to thread sync absent; fix!!!
            body.Dispose();
        }

        public static Collider CreateStaticCollider(CollisionShape shape, Matrix startTransform = default)
        {
            Vector3 localInertia = Vector3.Zero;
            RigidBody body = CreateBody(0, startTransform, shape, localInertia);
            return Collider.Create(body);
        }

        public static Collider CreateKinematicCollider(CollisionShape shape, Matrix startTransform = default)
        {
            Vector3 localInertia = Vector3.Zero;
            RigidBody body = CreateBody(0, startTransform, shape, localInertia);

            body.CollisionFlags = CollisionFlags.KinematicObject;
            body.ActivationState = ActivationState.DisableDeactivation;

            return Collider.Create(body);
        }

        public static Collider CreateDynamicCollider(CollisionShape shape, float mass, Matrix startTransform = default)
        {
            Vector3 localInertia = shape.CalculateLocalInertia(mass);
            RigidBody body = CreateBody(mass, startTransform, shape, localInertia);
            return Collider.Create(body);
        }

        public static RigidBody CreateBody(float mass, Matrix startTransform, CollisionShape shape, Vector3 localInertia)
        {
            var motionState = new DefaultMotionState(startTransform == default ? Matrix.Identity : startTransform);
            using (var rbInfo = new RigidBodyConstructionInfo(mass, motionState, shape, localInertia))
            {
                var body = new RigidBody(rbInfo);
                World.AddRigidBody(body);
                return body;
            }
        }

        public static void Dispose()  // TODO: can crash; examine!
        {
            // remove/dispose of constraints
            for (int i = World.NumConstraints - 1; i >= 0; i--)
            {
                TypedConstraint constraint = World.GetConstraint(i);
                World.RemoveConstraint(constraint);
                constraint.Dispose();
            }

            // remove the rigidbodies from the dynamics world and delete them
            for (int i = World.NumCollisionObjects - 1; i >= 0; i--)
            {
                CollisionObject obj = World.CollisionObjectArray[i];
                RigidBody body = obj as RigidBody;
                if (body != null && body.MotionState != null)
                {
                    body.MotionState.Dispose();
                }
                World.RemoveCollisionObject(obj);
                obj.Dispose();
            }

            // delete collision shapes
            foreach (CollisionShape shape in _collisionShapes)
            {
                shape.Dispose();
            }
            _collisionShapes.Clear();

            World.Dispose();
            _broadphase.Dispose();
            if (_dispatcher != null)
            {
                _dispatcher.Dispose();
            }
            _collisionConf.Dispose();
        }
    }
}