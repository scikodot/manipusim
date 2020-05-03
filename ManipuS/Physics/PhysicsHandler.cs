using BulletSharp;
using BulletSharp.Math;
using System.Collections.Generic;
using Vector3 = BulletSharp.Math.Vector3;

namespace Physics
{
    public static class PhysicsHandler
    {
        public static DiscreteDynamicsWorld World { get; }

        private static CollisionDispatcher _dispatcher;
        private static DbvtBroadphase _broadphase;
        private static List<CollisionShape> _collisionShapes = new List<CollisionShape>();
        private static CollisionConfiguration _collisionConf;

        static PhysicsHandler()
        {
            // collision configuration contains default setup for memory, collision setup
            _collisionConf = new DefaultCollisionConfiguration();
            _dispatcher = new CollisionDispatcher(_collisionConf);

            _broadphase = new DbvtBroadphase();
            World = new DiscreteDynamicsWorld(_dispatcher, _broadphase, null, _collisionConf);

            // create the ground
            var groundShape = new BoxShape(5, 0.125f, 5);
            _collisionShapes.Add(groundShape);
            CollisionObject ground = CreateStaticBody(Matrix.Identity, groundShape);
            ground.UserObject = "Ground";

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

        public static void Update(float elapsedTime)
        {
            World.StepSimulation(elapsedTime);
        }

        public static void ExitPhysics()
        {
            // remove/dispose constraints
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

        public static RigidBody CreateStaticBody(Matrix startTransform, CollisionShape shape)
        {
            Vector3 localInertia = Vector3.Zero;
            return CreateBody(0, startTransform, shape, localInertia);
        }

        public static RigidBody CreateKinematicBody(Matrix startTransform, CollisionShape shape)
        {
            Vector3 localInertia = Vector3.Zero;
            var body = CreateBody(0, startTransform, shape, localInertia);
            body.CollisionFlags = CollisionFlags.KinematicObject;
            body.ActivationState = ActivationState.DisableDeactivation;
            return body;
        }

        public static RigidBody CreateDynamicBody(float mass, Matrix startTransform, CollisionShape shape)
        {
            Vector3 localInertia = shape.CalculateLocalInertia(mass);
            return CreateBody(mass, startTransform, shape, localInertia);
        }

        private static RigidBody CreateBody(float mass, Matrix startTransform, CollisionShape shape, Vector3 localInertia)
        {
            var motionState = new DefaultMotionState(startTransform);
            using (var rbInfo = new RigidBodyConstructionInfo(mass, motionState, shape, localInertia))
            {
                var body = new RigidBody(rbInfo);
                World.AddRigidBody(body);
                return body;
            }
        }
    }
}