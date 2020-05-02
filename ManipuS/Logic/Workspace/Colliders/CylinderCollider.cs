using System;
using System.Collections.Generic;
using System.Numerics;

using BulletSharp;
using OpenTK.Graphics.OpenGL4;

using Graphics;
using Physics;

namespace Logic
{
    class CylinderCollider : ICollidable
    {
        private float _radius;
        private int _circleCount = 50;

        public Model Model { get; }
        public RigidBody Body { get; }

        public CylinderCollider(float radius, float extendDown, float extendUp)
        {
            uint offset = (uint)_circleCount + 2;

            var vertices = new MeshVertex[2 * offset];

            vertices[0] = new MeshVertex { Position = new OpenTK.Vector3(0, -extendDown, 0) };
            vertices[offset] = new MeshVertex { Position = new OpenTK.Vector3(0, extendUp, 0) };

            for (int i = 0; i <= _circleCount; i++)
            {
                float angle = 2 * (float)Math.PI * i / _circleCount;
                float cos = radius * (float)Math.Cos(angle);
                float sin = radius * (float)Math.Sin(angle);

                vertices[i + 1] = new MeshVertex { Position = new OpenTK.Vector3(cos, -extendDown, sin) };
                vertices[i + 1 + offset] = new MeshVertex { Position = new OpenTK.Vector3(cos, extendUp, sin) };
            }

            var indices = new List<uint>();

            // lower circle faces
            for (uint i = 0; i < _circleCount; i++)
            {
                indices.Add(0);
                indices.Add(i + 1);
                indices.Add(i + 2);
            }

            // upper circle faces
            for (uint i = 0; i < _circleCount; i++)
            {
                indices.Add(offset);
                indices.Add(i + 1 + offset);
                indices.Add(i + 2 + offset);
            }

            // side faces
            for (uint i = 0; i < _circleCount; i++)
            {
                indices.Add(i + 1);
                indices.Add(i + 1 + offset);
                indices.Add(i + 2);

                indices.Add(i + 1 + offset);
                indices.Add(i + 1 + offset + 1);
                indices.Add(i + 2);
            }

            Model = new Model(vertices, indices.ToArray(), MeshMaterial.Green);

            // create a rigid body
            Body = PhysicsHandler.CreateKinematicBody(new BulletSharp.Math.Matrix(
                1, 0, 0, 0,
                0, 1, 0, (extendUp - extendDown) / 2,
                0, 0, 1, 0,
                0, 0, 0, 1), new CylinderShape(radius, (extendDown + extendUp) / 2, radius));

            _radius = radius;
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
            Model.Render(shader, MeshMode.Wireframe);
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
