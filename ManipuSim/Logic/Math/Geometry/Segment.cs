﻿using System;
using System.Numerics;

namespace Logic
{
    public class Segment  // TODO: check usefulness; this class is rarely used
    {
        public Vector3 p1, p2;
        public float minX, maxX, minY, maxY;

        public Segment(Vector3 point1, Vector3 point2)
        {
            p1 = point1;
            p2 = point2;

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
}
