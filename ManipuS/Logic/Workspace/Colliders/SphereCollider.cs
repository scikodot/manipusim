using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

using OpenTK.Graphics.OpenGL4;

namespace Logic
{
    class SphereCollider : Collider
    {
        public float Radius;  // TODO: make some unified property like Size, instead of type-specific properties like Radius

        private uint levels = 15, pointsNum = 60;  // levels is always odd, pointsNum is always even
        public uint[] indicesLongitude;

        public SphereCollider(Vector3[] data) : base(data)
        {
            // central point
            Center = data.Sum() / data.Length;

            // radius
            Radius = Array.ConvertAll(data, x => x.DistanceTo(Center)).Max();

            // data
            Data = new Vector3[2 + levels * pointsNum];
            float y = Radius;
            Data[0] = y * Vector3.UnitY + Center;
            for (int i = 0; i < levels; i++)
            {
                y -= 2 * Radius / (levels + 1);
                double levelRadius = Math.Sqrt(Radius * Radius - y * y);
                for (int j = 0; j < pointsNum; j++)
                {
                    float angle = j * 2 * (float)Math.PI / pointsNum;
                    float x = (float)(levelRadius * Math.Cos(angle));
                    float z = (float)(levelRadius * Math.Sin(angle));
                    Data[1 + i * pointsNum + j] = new Vector3(x, y, z) + Center;
                }
            }
            y = -Radius;
            Data[Data.Length - 1] = y * Vector3.UnitY + Center;

            // defining indices to draw sphere
            List<uint> indices = new List<uint>();
            for (uint j = 0; j < pointsNum / 2; j++)
            {
                indices.Add(0);
                for (uint k = 0; k < levels; k++)
                {
                    indices.Add(j + k * pointsNum + 1);
                }
                indices.Add(levels * pointsNum + 1);
                for (uint k = 0; k < levels; k++)
                {
                    indices.Add(j + (levels - k) * pointsNum + 1 - pointsNum / 2);
                }
            }
            indicesLongitude = indices.ToArray();
        }

        public override bool Contains(Vector3 point)
        {
            return Center.DistanceTo(point) <= Radius;
        }

        public override Vector3 Extrude(Vector3 point)
        {
            Vector3 vec = point - Center;
            return Vector3.Normalize(vec) * Radius - vec;
        }

        // draw method for latitudinal circles
        public override void Draw(Graphics.Shader shader, Matrix4 model)
        {
            if (Entity == default)
                Entity = new Graphics.Entity(shader, Graphics.Utils.GL_Convert(Data, OpenTK.Graphics.Color4.Green), indicesLongitude);

            Entity.Display(model, () =>
            {
                for (int i = 0; i < levels; i++)
                {
                    GL.DrawArrays(PrimitiveType.LineLoop, i * (int)pointsNum + 1, (int)pointsNum);  // TODO: should be the same concept! replace with indices
                }

                GL.DrawElements(BeginMode.LineLoop, indicesLongitude.Length, DrawElementsType.UnsignedInt, 0);
            });
        }
    }
}
