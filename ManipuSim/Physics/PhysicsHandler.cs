using BulletSharp;
using BulletSharp.Math;
using BulletSharp.SoftBody;
using Graphics;
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

    public class PhysicsHandler
    {
        private readonly MainWindow _parent;
        private readonly object _worldSyncRoot = new();

        private readonly CollisionDispatcher _dispatcher;
        private readonly DbvtBroadphase _broadphase;
        private readonly List<CollisionShape> _collisionShapes = new();
        private readonly CollisionConfiguration _collisionConf;

        private const int MaxThreadCount = 64;
        private ConstraintSolverPoolMultiThreaded _solverPool;
        private SequentialImpulseConstraintSolverMultiThreaded _parallelSolver;
        private List<TaskScheduler> _schedulers = new();
        private int _currentScheduler = 0;

        public string[] RigidBodyTypes { get; } = Enum.GetNames(typeof(RigidBodyType));
        public DiscreteDynamicsWorld World { get; }

        public PhysicsHandler(MainWindow parent)
        {
            _parent = parent;

            //CreateSchedulers();
            //NextTaskScheduler();

            //using (var collisionConfigurationInfo = new DefaultCollisionConstructionInfo
            //{
            //    DefaultMaxPersistentManifoldPoolSize = 80000,
            //    DefaultMaxCollisionAlgorithmPoolSize = 80000
            //})
            //{
            //    _collisionConf = new DefaultCollisionConfiguration(collisionConfigurationInfo);
            //};
            //_dispatcher = new CollisionDispatcherMultiThreaded(_collisionConf);
            //_broadphase = new DbvtBroadphase();
            //_solverPool = new ConstraintSolverPoolMultiThreaded(MaxThreadCount);
            //_parallelSolver = new SequentialImpulseConstraintSolverMultiThreaded();
            //World = new DiscreteDynamicsWorldMultiThreaded(_dispatcher, _broadphase, _solverPool, _parallelSolver, _collisionConf);
            //World.SolverInfo.SolverMode = SolverModes.Simd | SolverModes.UseWarmStarting;

            // collision configuration contains default setup for memory, collision setup
            _collisionConf = new DefaultCollisionConfiguration();
            _dispatcher = new CollisionDispatcher(_collisionConf);

            _broadphase = new DbvtBroadphase();
            World = new DiscreteDynamicsWorld(_dispatcher, _broadphase, null, _collisionConf);

            World.Gravity = -10.0f * Vector3.UnitY;

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

        /*public void NextTaskScheduler()
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

        private void CreateSchedulers()
        {
            AddScheduler(Threads.GetSequentialTaskScheduler());
            AddScheduler(Threads.GetOpenMPTaskScheduler());
            AddScheduler(Threads.GetTbbTaskScheduler());
            AddScheduler(Threads.GetPplTaskScheduler());
        }

        private void AddScheduler(TaskScheduler scheduler)
        {
            if (scheduler != null)
            {
                _schedulers.Add(scheduler);
            }
        }*/        

        public void RayTestRef(ref Vector3 startWorld, ref Vector3 endWorld, ClosestRayResultCallback raycastCallback)
        {
            lock (_worldSyncRoot)
                World.RayTestRef(ref startWorld, ref endWorld, raycastCallback);
        }

        public bool ContactPairTest(RigidBody first, RigidBody second, CollisionCallback callback)
        {
            lock (_worldSyncRoot)
                World.ContactPairTest(first, second, callback);

            return callback.ContactDetected;
        }

        public bool ContactTest(RigidBody body, CollisionCallback callback)
        {
            lock (_worldSyncRoot)
                World.ContactTest(body, callback);

            return callback.ContactDetected;
        }

        public void Update(float elapsedTime)
        {
            lock (_worldSyncRoot)
                World.StepSimulation(elapsedTime);  // TODO: can crash, perhaps due to thread sync absent; fix!!!
        }

        public void AddRigidBody(RigidBody body)
        {
            lock (_worldSyncRoot)
                World.AddRigidBody(body);
        }

        public void RemoveRigidBody(RigidBody body)
        {
            lock (_worldSyncRoot)
                World.RemoveRigidBody(body);
        }

        /*public void CleanRigidBody(RigidBody body)
        {
            //// remove body from world
            //lock (_worldSyncRoot)
            //    World.RemoveRigidBody(body);

            // clear all forces and velocities for that body
            body.ClearForces();
            body.LinearVelocity = Vector3.Zero;
            body.AngularVelocity = Vector3.Zero;
        }*/

        public void DisposeRigidBody(RigidBody body)
        {
            // first, remove the body from the world
            RemoveRigidBody(body);  // [legacy] TODO: can crash, perhaps due to a lack of thread sync

            // then dipose of the body
            body?.Dispose();
        }

        public Collider CreateCollider(RigidBodyType type, CollisionShape shape, Matrix transform = default, float mass = 0)
        {
            var localInertia = mass == 0 ? Vector3.Zero : shape.CalculateLocalInertia(mass);
            var motionState = new DefaultMotionState(transform == default ? Matrix.Identity : transform);
            using var rbInfo = new RigidBodyConstructionInfo(mass, motionState, shape, localInertia);
            var body = new RigidBody(rbInfo);

            // kinematic bodies

            // add the body to the world
            AddRigidBody(body);

            // create a collider of the given shape
            Collider collider = shape.ShapeType switch
            {
                BroadphaseNativeType.BoxShape => new BoxCollider(body, type),
                BroadphaseNativeType.SphereShape => new SphereCollider(body, type),
                BroadphaseNativeType.CylinderShape => new CylinderCollider(body, type),
                BroadphaseNativeType.ConeShape => new ConeCollider(body, type),
                _ => throw new ArgumentException("Unexpected collision shape.")
            };

            // change the collider type to kinematic if in design mode
            if (_parent.Mode == InteractionMode.Design)
                collider.Body.CollisionFlags = CollisionFlags.StaticObject | CollisionFlags.KinematicObject;

            return collider;
        }

        public void Dispose()  // TODO: can crash; examine!
        {
            // remove/dispose of constraints
            for (int i = World.NumConstraints - 1; i >= 0; i--)
            {
                TypedConstraint constraint = World.GetConstraint(i);

                lock (_worldSyncRoot)
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
                
                lock (_worldSyncRoot)
                    World.RemoveCollisionObject(obj);

                obj.Dispose();
            }

            // delete collision shapes
            foreach (CollisionShape shape in _collisionShapes)
            {
                shape.Dispose();
            }
            _collisionShapes.Clear();

            lock (_worldSyncRoot)
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