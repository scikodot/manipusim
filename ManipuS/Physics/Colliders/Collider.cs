using System;
using System.Numerics;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;

using Graphics;

namespace Physics
{
    public abstract class Collider : IDisposable
    {
        public Model Model { get; protected set; }
        public RigidBody Body { get; protected set; }

        public CollisionCallback CollisionCallback { get; }

        public BroadphaseNativeType Shape => Body.CollisionShape.ShapeType;

        private RigidBodyType _type;
        public ref RigidBodyType Type => ref _type;

        public static Collider Create(RigidBody body)
        {
            Collider collider = default;
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
                collider._type = RigidBodyType.Static;
            else if (body.IsKinematicObject)
                collider._type = RigidBodyType.Kinematic;
            else
                collider._type = RigidBodyType.Dynamic;

            // convert collider to kinematic for design mode
            collider.Convert(RigidBodyType.Kinematic);

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

        public void UpdateState(ref OpenTK.Matrix4 state)
        {
            //var stateMatrix = new BulletSharp.Math.Matrix(
            //    state.M11, state.M21, state.M31, state.M41,
            //    state.M12, state.M22, state.M32, state.M42,
            //    state.M13, state.M23, state.M33, state.M43,
            //    state.M14, state.M24, state.M34, state.M44);

            //Body.MotionState.SetWorldTransform(ref stateMatrix);

            //Body.MotionState.GetWorldTransform(out stateMatrix);

            //Model.State = new OpenTK.Matrix4(
            //    stateMatrix.M11, stateMatrix.M12, stateMatrix.M13, stateMatrix.M14,
            //    stateMatrix.M21, stateMatrix.M22, stateMatrix.M23, stateMatrix.M24,
            //    stateMatrix.M31, stateMatrix.M32, stateMatrix.M33, stateMatrix.M34,
            //    stateMatrix.M41, stateMatrix.M42, stateMatrix.M43, stateMatrix.M44);

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

        public bool CollisionTest(Collider other)
        {
            //physics.World.ContactPairTest(Body, other.Body, CollisionCallback);
            return CollisionCallback.CollisionTest(Body, other.Body);
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