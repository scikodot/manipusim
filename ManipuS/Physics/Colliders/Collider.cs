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
            var state = body.MotionState.WorldTransform;
            OpenTK.Matrix4 stateMatrix = new OpenTK.Matrix4(
                state.M11, state.M21, state.M31, state.M41,
                state.M12, state.M22, state.M32, state.M42,
                state.M13, state.M23, state.M33, state.M43,
                state.M14, state.M24, state.M34, state.M44);
            collider.UpdateModel(ref stateMatrix);

            // convert collider to kinematic for design mode
            if (MainWindow.Mode == InteractionMode.Design)
                collider.Convert(RigidBodyType.Kinematic);

            // setup a callback for the collider
            collider.CollisionCallback = new CollisionCallback(collider.Body, null); 

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

        public void UpdateModel(ref OpenTK.Matrix4 stateOther)
        {
            var state = Matrix.Scaling(Body.CollisionShape.LocalScaling) * Body.MotionState.WorldTransform;
            OpenTK.Matrix4 stateMatrix = new OpenTK.Matrix4(
                state.M11, state.M21, state.M31, state.M41,
                state.M12, state.M22, state.M32, state.M42,
                state.M13, state.M23, state.M33, state.M43,
                state.M14, state.M24, state.M34, state.M44);

            Model.State = stateMatrix;
        }

        public abstract void Scale();

        public void Convert(RigidBodyType type)
        {
            switch (type)
            {
                case RigidBodyType.Static:
                    Body.CollisionFlags = CollisionFlags.StaticObject;
                    Body.ActivationState = ActivationState.ActiveTag;
                    break;
                case RigidBodyType.Kinematic:
                    Body.CollisionFlags = CollisionFlags.KinematicObject;
                    Body.ActivationState = ActivationState.DisableDeactivation;
                    break;
                case RigidBodyType.Dynamic:
                    Body.CollisionFlags = CollisionFlags.None;
                    Body.ActivationState = ActivationState.ActiveTag;
                    break;
            }
        }

        public bool CollisionPairTest(Collider other)
        {
            PhysicsHandler.ContactPairTest(Body, other.Body, CollisionCallback);
            if (CollisionCallback.IsCalled)
            {
                CollisionCallback.IsCalled = false;
                return true;
            }
            else
            {
                return false;
            }
        }

        public bool CollisionTest()
        {
            PhysicsHandler.ContactTest(Body, CollisionCallback);
            if (CollisionCallback.IsCalled)
            {
                CollisionCallback.IsCalled = false;
                return true;
            }
            else
            {
                return false;
            }
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
                    collider = PhysicsHandler.CreateDynamicCollider(Body.CollisionShape, 1.0f / Body.InvMass,Body.MotionState.WorldTransform);
                    break;
            }

            // disable collider to prevent it from producing collision impulses
            collider.Body.CollisionFlags |= CollisionFlags.NoContactResponse;

            // TODO: copy callback?

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