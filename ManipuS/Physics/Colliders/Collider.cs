using System;
using System.Numerics;

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

        public void UpdateModel(ref OpenTK.Matrix4 state)
        {
            Model.State = state;
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
            PhysicsHandler.World.ContactPairTest(Body, other.Body, CollisionCallback);
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
            PhysicsHandler.World.ContactTest(Body, CollisionCallback);  // TODO: maintain thread sync!
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