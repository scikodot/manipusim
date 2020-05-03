using System;
using System.Numerics;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Physics;

using Vector3 = System.Numerics.Vector3;

namespace Logic
{
    class SphereCollider : ICollidable
    {
        private readonly float _radius;

        public Model Model { get; }
        public RigidBody Body { get; private set; }

        private SphereCollider(float radius, uint stackCount, uint sectorCount, out Matrix state, out SphereShape bodyShape)
        {
            Model = Primitives.Sphere(radius, stackCount, sectorCount, MeshMaterial.Green);

            state = Matrix.Identity;

            bodyShape = new SphereShape(radius);

            _radius = radius;
        }

        public static SphereCollider CreateStatic(float radius, uint stackCount, uint sectorCount)
        {
            return new SphereCollider(radius, stackCount, sectorCount, out Matrix state, out SphereShape bodyShape)
            {
                Body = PhysicsHandler.CreateStaticBody(state, bodyShape)
            };
        }

        public static SphereCollider CreateKinematic(float radius, uint stackCount, uint sectorCount)
        {
            return new SphereCollider(radius, stackCount, sectorCount, out Matrix state, out SphereShape bodyShape)
            {
                Body = PhysicsHandler.CreateKinematicBody(state, bodyShape)
            };
        }

        public static SphereCollider CreateDynamic(float radius, uint stackCount, uint sectorCount, float mass)
        {
            return new SphereCollider(radius, stackCount, sectorCount, out Matrix state, out SphereShape bodyShape)
            {
                Body = PhysicsHandler.CreateDynamicBody(mass, state, bodyShape)
            };
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
