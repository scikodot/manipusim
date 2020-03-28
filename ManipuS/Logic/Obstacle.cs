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
        Point[] Data { get; set; }
        Point Center { get; set; }

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
        public Point[] Data { get; set; }
        public Point Center { get; set; }

        public Box(Point[] obst)
        {
            // retrieving boundary points
            double Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0, Zmin = 0, Zmax = 0;
            for (int i = 0; i < obst.Length; i++)
            {
                if (i == 0)
                {
                    Xmin = Xmax = obst[i].x;
                    Ymin = Ymax = obst[i].y;
                    Zmin = Zmax = obst[i].z;
                }
                else
                {
                    if (obst[i].x < Xmin)
                        Xmin = obst[i].x;
                    if (obst[i].x > Xmax)
                        Xmax = obst[i].x;
                    if (obst[i].y < Ymin)
                        Ymin = obst[i].y;
                    if (obst[i].y > Ymax)
                        Ymax = obst[i].y;
                    if (obst[i].z > Zmax)
                        Zmax = obst[i].z;
                    if (obst[i].z < Zmin)
                        Zmin = obst[i].z;
                }
            }

            // data
            Data = new Point[8]
            {
                        new Point(Xmin, Ymin, Zmin),
                        new Point(Xmax, Ymin, Zmin),
                        new Point(Xmax, Ymin, Zmax),
                        new Point(Xmin, Ymin, Zmax),
                        new Point(Xmin, Ymax, Zmin),
                        new Point(Xmax, Ymax, Zmin),
                        new Point(Xmax, Ymax, Zmax),
                        new Point(Xmin, Ymax, Zmax)
            };

            // central point
            Center = new Point((Xmax + Xmin) / 2, (Ymax + Ymin) / 2, (Zmax + Zmin) / 2);
        }

        public void Draw()
        {
            GL.DrawElements(BeginMode.LineStrip, 16, DrawElementsType.UnsignedInt, 0);
        }
    }

    class Sphere : ICollider
    {
        public ColliderShape Shape { get { return ColliderShape.Sphere; } }
        public Point[] Data { get; set; }
        public Point Center { get; set; }
        public double Radius;

        private uint levels = 15, pointsNum = 60;  // levels is always odd, pointsNum is always even
        public uint[] indicesLongitude;

        public Sphere(Point[] obst)
        {
            // central point
            Center = Point.Zero;
            for (int i = 0; i < obst.Length; i++)
            {
                Center += obst[i];
            }
            Center /= obst.Length;

            // radius
            Radius = 0;
            for (int i = 0; i < obst.Length; i++)
            {
                double rNew = obst[i].DistanceTo(Center);
                if (rNew > Radius)
                    Radius = rNew;
            }

            // data
            Data = new Point[2 + levels * pointsNum];
            double y = Radius;
            Data[0] = new Point(0, y, 0) + Center;
            for (int i = 0; i < levels; i++)
            {
                y -= 2 * Radius / (levels + 1);
                double levelRadius = Math.Sqrt(Radius * Radius - y * y);
                for (int j = 0; j < pointsNum; j++)
                {
                    double angle = j * 2 * Math.PI / pointsNum;
                    double x = levelRadius * Math.Cos(angle),
                           z = levelRadius * Math.Sin(angle);
                    Data[1 + i * pointsNum + j] = new Point(x, y, z) + Center;
                }
            }
            y = -Radius;
            Data[Data.Length - 1] = new Point(0, y, 0) + Center;

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

        // draw method for latitudinal circles
        public void Draw()
        {
            for (int i = 0; i < levels; i++)
            {
                GL.DrawArrays(PrimitiveType.LineLoop, i * (int)pointsNum + 1, (int)pointsNum);  // TODO: should be the same concept! replace with indices
            }
        }

        // draw method for longitudinal circles
        public void DrawLongitudes()
        {
            GL.DrawElements(BeginMode.LineLoop, indicesLongitude.Length, DrawElementsType.UnsignedInt, 0);
        }
    }

    class Obstacle
    {
        public Point[] Data { get; set; }
        public ICollider Collider;

        public Obstacle(Point[] data, ColliderShape shape)
        {
            Data = new Point[data.Length];
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

        public bool Contains(Point p)
        {
            switch (Collider.Shape)
            {
                case ColliderShape.Box:
                    if (p.x >= Collider.Data[0].x && p.x <= Collider.Data[1].x &&
                        p.y >= Collider.Data[0].y && p.y <= Collider.Data[4].y &&
                        p.z >= Collider.Data[0].z && p.z <= Collider.Data[2].z)
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

        public void Move(Vector direction, float distance)
        {
            //for (int i = 0; i < Data.Length; i++)
            //{
            //    Data[i] += direction.Normalized * distance;
            //}

            Collider.Center += direction.Normalized * distance;
        }
    }
}
