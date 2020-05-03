using System.Numerics;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;

using Graphics;
using Physics;

namespace Logic
{
    public enum ColliderShape
    {
        Box,
        Sphere
    }

    public abstract class Collider  // TODO: make abstract and provide unified implementations for Render and UpdateState; see BoxCollider
    {
        public Model Model;
        public RigidBody Body;

        public CollisionCallback CollisionCallback;

        public virtual bool Contains(Vector3 point)
        {
            return default;
        }

        public virtual Vector3 Extrude(Vector3 point)
        {
            return default;
        }

        public virtual void Render(Shader shader)
        {
            Model.Render(shader, MeshMode.Solid, () =>
            {
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
                GL.DrawElements(PrimitiveType.Triangles, Model.Meshes[0].Indices.Length, DrawElementsType.UnsignedInt, 0);
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            });
        }

        public virtual void UpdateState(ref OpenTK.Matrix4 state)
        {
            var stateMatrix = new BulletSharp.Math.Matrix(
                state.M11, state.M21, state.M31, state.M41,
                state.M12, state.M22, state.M32, state.M42,
                state.M13, state.M23, state.M33, state.M43,
                state.M14, state.M24, state.M34, state.M44);

            Body.MotionState.SetWorldTransform(ref stateMatrix);

            Body.MotionState.GetWorldTransform(out stateMatrix);

            Model.State = new OpenTK.Matrix4(
                stateMatrix.M11, stateMatrix.M12, stateMatrix.M13, stateMatrix.M14,
                stateMatrix.M21, stateMatrix.M22, stateMatrix.M23, stateMatrix.M24,
                stateMatrix.M31, stateMatrix.M32, stateMatrix.M33, stateMatrix.M34,
                stateMatrix.M41, stateMatrix.M42, stateMatrix.M43, stateMatrix.M44);
        }

        public bool CollisionTest(Collider other)
        {
            //physics.World.ContactPairTest(Body, other.Body, CollisionCallback);
            return CollisionCallback.CollisionTest(Body, other.Body);
        }
    }
}