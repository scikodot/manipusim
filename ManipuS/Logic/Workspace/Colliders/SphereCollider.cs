using System;
using System.Numerics;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;

using Graphics;
using Physics;

namespace Logic
{
    class SphereCollider : ICollidable
    {
        private float _radius;
        private int _circleCount = 50;

        public Model Model { get; }
        public RigidBody Body { get; }

        public SphereCollider(float radius, uint stackCount, uint sectorCount)
        {
            Model = Primitives.Sphere(radius, stackCount, sectorCount, MeshMaterial.Green);

            // create a rigid body
            Body = PhysicsHandler.CreateKinematicBody(BulletSharp.Math.Matrix.Identity, new SphereShape(radius));

            _radius = radius;
        }

        public bool Contains(Vector3 point)
        {
            var center = Body.CenterOfMassPosition;
            return new Vector3(center.X, center.Y, center.Z).DistanceTo(point) <= _radius;
        }

        public Vector3 Extrude(Vector3 point)
        {
            var center = Body.CenterOfMassPosition;
            Vector3 vec = point - new Vector3(center.X, center.Y, center.Z);
            return Vector3.Normalize(vec) * _radius - vec;
        }

        public void Render(Shader shader)
        {
            Model.Render(shader, MeshMode.Solid, () =>
            {
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
                GL.DrawElements(PrimitiveType.Triangles, Model.Meshes[0].Indices.Length, DrawElementsType.UnsignedInt, 0);
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
