using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK;
using OpenTK.Graphics.OpenGL4;
using Graphics;
using Logic.PathPlanning;

namespace Logic
{   
    public interface ICollider
    {
        ColliderShape Shape { get; }
        Vector3[] Data { get; set; }
        Vector3 Center { get; set; }

        void Draw();
    }

    public enum ColliderShape
    {
        Box,
        Sphere
    }

    class Box : ICollider
    {
        public ColliderShape Shape { get { return ColliderShape.Box; } }
        public Vector3[] Data { get; set; }
        public Vector3 Center { get; set; }

        public Box(Vector3[] obst)
        {
            // retrieving boundary Vector3s
            float Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0, Zmin = 0, Zmax = 0;
            for (int i = 0; i < obst.Length; i++)
            {
                if (i == 0)
                {
                    Xmin = Xmax = obst[i].X;
                    Ymin = Ymax = obst[i].Y;
                    Zmin = Zmax = obst[i].Z;
                }
                else
                {
                    if (obst[i].X < Xmin)
                        Xmin = obst[i].X;
                    if (obst[i].X > Xmax)
                        Xmax = obst[i].X;
                    if (obst[i].Y < Ymin)
                        Ymin = obst[i].Y;
                    if (obst[i].Y > Ymax)
                        Ymax = obst[i].Y;
                    if (obst[i].Z > Zmax)
                        Zmax = obst[i].Z;
                    if (obst[i].Z < Zmin)
                        Zmin = obst[i].Z;
                }
            }

            // data
            Data = new Vector3[8]
            {
                        new Vector3(Xmin, Ymin, Zmin),
                        new Vector3(Xmax, Ymin, Zmin),
                        new Vector3(Xmax, Ymin, Zmax),
                        new Vector3(Xmin, Ymin, Zmax),
                        new Vector3(Xmin, Ymax, Zmin),
                        new Vector3(Xmax, Ymax, Zmin),
                        new Vector3(Xmax, Ymax, Zmax),
                        new Vector3(Xmin, Ymax, Zmax)
            };

            // central Vector3
            Center = new Vector3((Xmax + Xmin) / 2, (Ymax + Ymin) / 2, (Zmax + Zmin) / 2);
        }

        public void Draw()
        {
            GL.DrawElements(BeginMode.LineStrip, 16, DrawElementsType.UnsignedInt, 0);
        }
    }

    class Sphere : ICollider
    {
        public ColliderShape Shape { get { return ColliderShape.Sphere; } }
        public Vector3[] Data { get; set; }
        public Vector3 Center { get; set; }
        public float Radius;

        private uint levels = 15, Vector3sNum = 60;  // levels is always odd, Vector3sNum is always even
        public uint[] indicesLongitude;

        public Sphere(Vector3[] obst)
        {
            // central Vector3
            Center = Vector3.Zero;
            for (int i = 0; i < obst.Length; i++)
            {
                Center += obst[i];
            }
            Center /= obst.Length;

            // radius
            Radius = 0;
            for (int i = 0; i < obst.Length; i++)
            {
                float rNew = obst[i].DistanceTo(Center);
                if (rNew > Radius)
                    Radius = rNew;
            }

            // data
            Data = new Vector3[2 + levels * Vector3sNum];
            float y = Radius;
            Data[0] = new Vector3(0, y, 0) + Center;
            for (int i = 0; i < levels; i++)
            {
                y -= 2 * Radius / (levels + 1);
                float levelRadius = (float)Math.Sqrt(Radius * Radius - y * y);
                for (int j = 0; j < Vector3sNum; j++)
                {
                    float angle = j * 2 * (float)Math.PI / Vector3sNum;
                    float x = levelRadius * (float)Math.Cos(angle),
                           z = levelRadius * (float)Math.Sin(angle);
                    Data[1 + i * Vector3sNum + j] = new Vector3(x, y, z) + Center;
                }
            }
            y = -Radius;
            Data[Data.Length - 1] = new Vector3(0, y, 0) + Center;

            // defining indices to draw sphere
            List<uint> indices = new List<uint>();
            for (uint j = 0; j < Vector3sNum / 2; j++)
            {
                indices.Add(0);
                for (uint k = 0; k < levels; k++)
                {
                    indices.Add(j + k * Vector3sNum + 1);
                }
                indices.Add(levels * Vector3sNum + 1);
                for (uint k = 0; k < levels; k++)
                {
                    indices.Add(j + (levels - k) * Vector3sNum + 1 - Vector3sNum / 2);
                }
            }
            indicesLongitude = indices.ToArray();
        }

        // draw method for latitudinal circles
        public void Draw()
        {
            for (int i = 0; i < levels; i++)
            {
                GL.DrawArrays(PrimitiveType.LineLoop, i * (int)Vector3sNum + 1, (int)Vector3sNum);  // TODO: should be the same concept! replace with indices
            }
        }

        // draw method for longitudinal circles
        public void DrawLongitudes()
        {
            GL.DrawElements(BeginMode.LineLoop, indicesLongitude.Length, DrawElementsType.UnsignedInt, 0);
        }
    }

    public class Obstacle
    {
        public Vector3[] Data { get; set; }
        public ICollider Collider;

        public Obstacle(Vector3[] data, ColliderShape shape)
        {
            Data = new Vector3[data.Length];
            Array.Copy(data, Data, data.Length);
            
            switch (shape)
            {
                case ColliderShape.Box:
                    Collider = new Box(Data);
                    break;
                case ColliderShape.Sphere:
                    Collider = new Sphere(Data);
                    break;
            }
        }

        public bool Contains(Vector3 p)
        {
            switch (Collider.Shape)
            {
                case ColliderShape.Box:
                    if (p.X >= Collider.Data[0].X && p.X <= Collider.Data[1].X &&
                        p.Y >= Collider.Data[0].Y && p.Y <= Collider.Data[4].Y &&
                        p.Z >= Collider.Data[0].Z && p.Z <= Collider.Data[2].Z)
                        return true;
                    else
                        return false;
                case ColliderShape.Sphere:
                    if (Collider.Center.DistanceTo(p) <= ((Sphere)Collider).Radius)
                        return true;
                    else
                        return false;
            }

            return false;
        }

        public void Move(Vector3 direction, float distance)
        {
            //for (int i = 0; i < Data.Length; i++)
            //{
            //    Data[i] += direction.Normalized * distance;
            //}

            Collider.Center += direction.Normalized * distance;
        }
    }
}
