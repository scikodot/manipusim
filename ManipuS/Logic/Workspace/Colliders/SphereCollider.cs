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
        //private uint levels = 15, pointsNum = 60;  // levels is always odd, pointsNum is always even
        //public uint[] indicesLongitude;

        private float _radius;
        private int _circleCount = 50;

        public Model Model { get; }
        public RigidBody Body { get; }

        //public SphereCollider(Vector3[] data) : base(data)
        //{
        //    // central point
        //    Center = data.Sum() / data.Length;

        //    // radius
        //    Radius = Array.ConvertAll(data, x => x.DistanceTo(Center)).Max();

        //    // data
        //    Data = new Vector3[2 + levels * pointsNum];
        //    float y = Radius;
        //    Data[0] = y * Vector3.UnitY + Center;
        //    for (int i = 0; i < levels; i++)
        //    {
        //        y -= 2 * Radius / (levels + 1);
        //        double levelRadius = Math.Sqrt(Radius * Radius - y * y);
        //        for (int j = 0; j < pointsNum; j++)
        //        {
        //            float angle = j * 2 * (float)Math.PI / pointsNum;
        //            float x = (float)(levelRadius * Math.Cos(angle));
        //            float z = (float)(levelRadius * Math.Sin(angle));
        //            Data[1 + i * pointsNum + j] = new Vector3(x, y, z) + Center;
        //        }
        //    }
        //    y = -Radius;
        //    Data[Data.Length - 1] = y * Vector3.UnitY + Center;

        //    // defining indices to draw sphere
        //    List<uint> indices = new List<uint>();
        //    for (uint j = 0; j < pointsNum / 2; j++)
        //    {
        //        indices.Add(0);
        //        for (uint k = 0; k < levels; k++)
        //        {
        //            indices.Add(j + k * pointsNum + 1);
        //        }
        //        indices.Add(levels * pointsNum + 1);
        //        for (uint k = 0; k < levels; k++)
        //        {
        //            indices.Add(j + (levels - k) * pointsNum + 1 - pointsNum / 2);
        //        }
        //    }
        //    indicesLongitude = indices.ToArray();
        //}

        public SphereCollider(float radius)
        {
            var vertices = new MeshVertex[3 * (_circleCount + 1)];
            for (int i = 0; i <= _circleCount; i++)
            {
                float angle = 2 * (float)Math.PI * i / _circleCount;
                float cos = radius * (float)Math.Cos(angle);
                float sin = radius * (float)Math.Sin(angle);

                vertices[i] = new MeshVertex { Position = new OpenTK.Vector3(cos, sin, 0) };
                vertices[i + _circleCount + 1] = new MeshVertex { Position = new OpenTK.Vector3(0, cos, sin) };
                vertices[i + 2 * (_circleCount + 1)] = new MeshVertex { Position = new OpenTK.Vector3(sin, 0, cos) };
            }

            Model = new Model(vertices, material: MeshMaterial.Green);

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

        public void Render(Shader shader, ref OpenTK.Matrix4 state)
        {
            Model.State = state;
            Model.Render(shader, MeshMode.Solid, () =>
            {
                GL.DrawArrays(PrimitiveType.LineStrip, 0, _circleCount + 1);
                GL.DrawArrays(PrimitiveType.LineStrip, _circleCount + 1, _circleCount + 1);
                GL.DrawArrays(PrimitiveType.LineStrip, 2 * (_circleCount + 1), _circleCount + 1);
                //for (int i = 0; i < levels; i++)
                //{
                //    GL.DrawArrays(PrimitiveType.LineLoop, i * (int)pointsNum + 1, (int)pointsNum);  // TODO: should be the same concept! replace with indices
                //}

                //GL.DrawElements(BeginMode.LineLoop, indicesLongitude.Length, DrawElementsType.UnsignedInt, 0);
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
