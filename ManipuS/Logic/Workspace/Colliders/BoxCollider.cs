using System.Numerics;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;

using Graphics;
using Physics;

namespace Logic
{
    class BoxCollider : ICollidable
    {
        private Vector3 _size;

        public Model Model { get; }
        public RigidBody Body { get; }

        public BoxCollider(float halfX, float halfY, float halfZ)
        {
            Model = Primitives.Cube(halfX, halfY, halfZ, MeshMaterial.Green);

            // create a rigid body
            Body = PhysicsHandler.CreateStaticBody(BulletSharp.Math.Matrix.Identity, new BoxShape(halfX, halfY, halfZ));

            _size = 2 * new Vector3(halfX, halfY, halfZ);
        }

        public bool Contains(Vector3 point)
        {
            var center = Body.CenterOfMassPosition;
            return point.X >= center.X - _size.X && point.X <= center.X + _size.X &&
                   point.Y >= center.Y - _size.Y && point.Y <= center.Y + _size.Y &&
                   point.Z >= center.Z - _size.Z && point.Z <= center.Z + _size.Z;
        }

        public Vector3 Extrude(Vector3 point)  // TODO: this method behaves weirdly; repair
        {
            var center = Body.CenterOfMassPosition;
            Vector3 vec = point - new Vector3(center.X, center.Y, center.Z);
            Vector3 diff = _size - vec;
            float ratio;
            if (diff.X > diff.Y)
            {
                if (diff.X > diff.Z)
                {
                    if (diff.Y > diff.Z)
                        ratio = diff.Z / vec.Z;
                    else
                        ratio = diff.Y / vec.Y;
                }
                else
                    ratio = diff.Y / vec.Y;
            }
            else
            {
                if (diff.Y > diff.Z)
                {
                    if (diff.X > diff.Z)
                        ratio = diff.Z / vec.Z;
                    else
                        ratio = diff.X / vec.X;
                }
                else
                    ratio = diff.X / vec.X;
            }

            return vec * ratio;
        }

        public void Render(Shader shader)  // TODO: perhaps make methods Render and UpdateState unified? this can be achieved with the abstract class
        {
            Model.Render(shader, MeshMode.Solid, () =>
            {
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
                GL.DrawElements(BeginMode.Triangles, Model.Meshes[0].Indices.Length, DrawElementsType.UnsignedInt, 0);
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            });
        }

        public void UpdateState(ref OpenTK.Matrix4 state)
        {
            Model.State = state;

            var stateMatrix = new BulletSharp.Math.Matrix(
                state.M11, state.M12, state.M13, state.M14,
                state.M21, state.M22, state.M23, state.M24,
                state.M31, state.M32, state.M33, state.M34,
                state.M41, state.M42, state.M43, state.M44);
            stateMatrix.Transpose();
            Body.MotionState.SetWorldTransform(ref stateMatrix);
        }
    }
}
