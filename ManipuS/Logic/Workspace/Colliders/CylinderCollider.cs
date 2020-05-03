using System;
using System.Collections.Generic;
using System.Numerics;

using BulletSharp;
using BulletSharp.Math;
using OpenTK.Graphics.OpenGL4;

using Graphics;
using Physics;

using Vector3 = System.Numerics.Vector3;

namespace Logic
{
    class CylinderCollider : ICollidable
    {
        private float _radius;

        public Model Model { get; }
        public RigidBody Body { get; private set; }

        private CylinderCollider(float radius, float extendDown, float extendUp, int circleCount, out Matrix state, out CylinderShape bodyShape)
        {
            Model = Primitives.Cylinder(radius, extendDown, extendUp, circleCount, MeshMaterial.Green);

            state = Matrix.Translation(0, (extendUp - extendDown) / 2, 0);

            bodyShape = new CylinderShape(radius, (extendDown + extendUp) / 2, radius);

            _radius = radius;
        }

        public static CylinderCollider CreateStatic(float radius, float extendDown, float extendUp, int circleCount)
        {
            return new CylinderCollider(radius, extendDown, extendUp, circleCount, out Matrix state, out CylinderShape bodyShape)
            {
                Body = PhysicsHandler.CreateStaticBody(state, bodyShape)
            };
        }

        public static CylinderCollider CreateKinematic(float radius, float extendDown, float extendUp, int circleCount)
        {
            return new CylinderCollider(radius, extendDown, extendUp, circleCount, out Matrix state, out CylinderShape bodyShape)
            {
                Body = PhysicsHandler.CreateKinematicBody(state, bodyShape)
            };
        }

        public static CylinderCollider CreateDynamic(float radius, float extendDown, float extendUp, int circleCount, float mass)
        {
            return new CylinderCollider(radius, extendDown, extendUp, circleCount, out Matrix state, out CylinderShape bodyShape)
            {
                Body = PhysicsHandler.CreateDynamicBody(mass, state, bodyShape)
            };
        }

        public bool Contains(Vector3 point)
        {
            return default;
        }

        public Vector3 Extrude(Vector3 point)
        {
            return default;
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

            //var stateMatrix = new Matrix(
            //    state.M11, state.M12, state.M13, state.M14,
            //    state.M21, state.M22, state.M23, state.M24,
            //    state.M31, state.M32, state.M33, state.M34,
            //    state.M41, state.M42, state.M43, state.M44);
            //stateMatrix.Transpose();
            //Body.MotionState.SetWorldTransform(ref stateMatrix);
        }
    }
}
