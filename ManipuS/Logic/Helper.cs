using System;
using System.Collections.Generic;
using System.Linq;
using OpenTK;

namespace Logic
{
    public class Segment
    {
        public Vector3 p1, p2;
        public float minX, maxX, minY, maxY;

        public Segment(Vector3 Vector31, Vector3 Vector32)
        {
            p1 = Vector31;
            p2 = Vector32;

            minX = Math.Min(p1.X, p2.X);
            maxX = Math.Max(p1.X, p2.X);
            minY = Math.Min(p1.Y, p2.Y);
            maxY = Math.Max(p1.Y, p2.Y);
        }

        public Vector3[] Discretize(int segments)
        {
            Vector3[] pos = new Vector3[segments + 1];
            for (int j = 0; j < segments + 1; j++)
            {
                pos[j] = new Vector3
                (
                    p1.X + j * (p2.X - p1.X) / segments,
                    p1.Y + j * (p2.Y - p1.Y) / segments,
                    p1.Z + j * (p2.Z - p1.Z) / segments
                );
            }
            return pos;
        }
    }

    public class Attractor  // TODO: maybe struct would be better?
    {
        public Vector3 Center;
        public float Weight;
        public Vector3[] Area;
        public float Radius;
        public float InliersCount;

        public Attractor(Vector3 center, float weight, Vector3[] area, float radius)
        {
            Center = center;
            Weight = weight;
            Area = area;
            Radius = radius;
            InliersCount = 0;
        }
    }

    public static class Misc
    {
        public static float BoxMullerTransform(Random rng, float mu, float sigma)
        {
            double phi = rng.NextDouble();
            double r = 1 - rng.NextDouble();  // exclude zero
            float z = (float)(Math.Cos(2 * Math.PI * phi) * Math.Sqrt(-2 * Math.Log(r)));
            return mu + sigma * z;
        }
    }

    public static class Primitives
    {
        public static Vector3[] Sphere(float radius, Vector3 center, int pointsNum, Random rng)
        {
            Vector3[] data = new Vector3[pointsNum];
            double x, yPos, y, zPos, z;
            for (int i = 0; i < pointsNum; i++)
            {
                x = radius * (2 * rng.NextDouble() - 1);
                yPos = Math.Sqrt(radius * radius - x * x);
                y = yPos * (2 * rng.NextDouble() - 1);
                zPos = Math.Sqrt(yPos * yPos - y * y);
                z = rng.Next(0, 2) == 0 ? -zPos : zPos;

                data[i] = new Vector3((float)x, (float)y, (float)z) + center;
            }

            return data;
        }
    }
}
