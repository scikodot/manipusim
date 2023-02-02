using System;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;

using Graphics;
using BulletSharp.Math;
using Vector3 = System.Numerics.Vector3;

namespace Physics
{
    public abstract class Collider : IDisposable
    {
        private static float _defaultMass = 1f;

        public Model Model { get; protected set; }
        public RigidBody Body { get; protected set; }

        public CollisionCallback CollisionCallback { get; protected set; }

        public BroadphaseNativeType Shape => Body.CollisionShape.ShapeType;
        public RigidBodyType Type { get; set; }

        public static Collider Create(RigidBody body)
        {
            Collider collider = default;

            // choose the type of the collider
            switch (body.CollisionShape.ShapeType)
            {
                case BroadphaseNativeType.BoxShape:
                    collider = new BoxCollider(body);
                    break;
                case BroadphaseNativeType.SphereShape:
                    collider = new SphereCollider(body);
                    break;
                case BroadphaseNativeType.CylinderShape:
                    collider = new CylinderCollider(body);
                    break;
                case BroadphaseNativeType.ConeShape:
                    collider = new ConeCollider(body);
                    break;
            }

            // memoize collider type
            if (body.IsStaticObject)
                collider.Type = RigidBodyType.Static;
            else if (body.IsKinematicObject)
                collider.Type = RigidBodyType.Kinematic;
            else
                collider.Type = RigidBodyType.Dynamic;

            // apply initial transformation to the collider model
            collider.UpdateModel();

            // setup a callback for the collider
            collider.CollisionCallback = new CollisionCallback(collider.Body, null);

            // convert collider to kinematic type if in design mode
            if (MainWindow.Mode == InteractionMode.Design)
                collider.Body.CollisionFlags = CollisionFlags.StaticObject | CollisionFlags.KinematicObject;

            return collider;
        }

        public abstract bool Contains(Vector3 point);

        public abstract Vector3 Extrude(Vector3 point);

        public void Render(Shader shader)
        {
            Model.Render(shader, () =>
            {
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
                GL.DrawElements(PrimitiveType.Triangles, Model.Meshes[0].Indices.Length, DrawElementsType.UnsignedInt, 0);
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            });
        }

        public void UpdateModel()
        {
            var state = Matrix.Scaling(Body.CollisionShape.LocalScaling) * Body.MotionState.WorldTransform;
            Model.State = state.TopOpenTK();
        }

        public abstract void Scale();

        public void Reset()
        {
            //PhysicsHandler.CleanRigidBody(Body);
        }

        public void Convert(RigidBodyType type, float mass)
        {
            PhysicsHandler.RemoveRigidBody(Body);

            // set mass properties
            if (type != RigidBodyType.Dynamic)
                mass = 0;

            Body.SetMassProps(mass, Body.CollisionShape.CalculateLocalInertia(mass));

            // set body type
            switch (type)
            {
                case RigidBodyType.Static:
                    Body.CollisionFlags = CollisionFlags.StaticObject;
                    Body.ForceActivationState(ActivationState.ActiveTag);
                    break;
                case RigidBodyType.Kinematic:
                    Body.CollisionFlags = CollisionFlags.StaticObject | CollisionFlags.KinematicObject;
                    Body.ForceActivationState(ActivationState.DisableDeactivation);
                    break;
                case RigidBodyType.Dynamic:
                    Body.CollisionFlags = CollisionFlags.None;
                    Body.ForceActivationState(ActivationState.ActiveTag);
                    Body.Activate();
                    break;
            }

            PhysicsHandler.AddRigidBody(Body);
        }

        public bool CollisionPairTest(Collider other)
        {
            return PhysicsHandler.ContactPairTest(Body, other.Body, CollisionCallback);
        }

        public bool CollisionTest()
        {
            return PhysicsHandler.ContactTest(Body, CollisionCallback);
        }

        //public void AttachGhost()
        //{
        //    // add a callback for the ghost object to detect collisions
        //    PhysicsHandler.World.Broadphase.OverlappingPairCache.SetInternalGhostPairCallback(new GhostPairCallback());

        //    // attach a ghost object to the collider
        //    Ghost = new GhostObject();
        //    Ghost.CollisionShape = Body.CollisionShape;
        //    Ghost.WorldTransform = Body.WorldTransform;
        //    Ghost.CollisionFlags |= CollisionFlags.NoContactResponse;

        //    // add the ghost object to the world
        //    PhysicsHandler.World.AddCollisionObject(Ghost, CollisionFilterGroups.SensorTrigger, CollisionFilterGroups.AllFilter);
        //}

        //public bool CollisionTestGhost()
        //{
        //    if (Ghost != null)
        //    {
        //        // if base ---> ignore

        //        // if prev/next neighbour ---> ignore

        //        // if any original manipulator collider ---> ignore

        //        Console.WriteLine($"Collisions: {Ghost.NumOverlappingObjects}");
        //        return Ghost.NumOverlappingObjects > 0;
        //    }
        //    else
        //        throw new ArgumentNullException("The ghost object is undefined!", "Ghost");
        //}

        public Collider DeepCopy()
        {
            // body cannot be cloned directly, so the only way is to re-initialize it
            Collider collider = default;
            switch (Type)
            {
                case RigidBodyType.Static:
                    collider = PhysicsHandler.CreateStaticCollider(Body.CollisionShape, Body.MotionState.WorldTransform);
                    break;
                case RigidBodyType.Kinematic:
                    collider = PhysicsHandler.CreateKinematicCollider(Body.CollisionShape, Body.MotionState.WorldTransform);
                    break;
                case RigidBodyType.Dynamic:
                    collider = PhysicsHandler.CreateDynamicCollider(Body.CollisionShape, 1.0f / Body.InvMass, Body.MotionState.WorldTransform);
                    break;
            }

            // disable collider to prevent it from producing collision impulses
            collider.Body.CollisionFlags |= CollisionFlags.NoContactResponse;

            // copy callback
            collider.CollisionCallback = new CollisionCallback(collider.Body, null);

            return collider;
        }

        public void Dispose()
        {
            // clear managed resources
            Model.Dispose();
            Body.DisposeFromWorld();

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}