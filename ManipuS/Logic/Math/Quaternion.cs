using System;
using OpenTK;

namespace Logic
{
    public struct Quaternion
    {
        public Vector3 XYZ { get; }
        public float W { get; }

        public float X => XYZ.X;
        public float Y => XYZ.Y;
        public float Z => XYZ.Z;

        public static Quaternion Null => new Quaternion(0, 0, 0, 0);

        public static Quaternion Zero => new Quaternion(0, 0, 0, 1);

        public float Length => (float)Math.Sqrt(XYZ.LengthSquared + W * W);

        public float LengthSquared => XYZ.LengthSquared + W * W;

        public Quaternion Conjugate => new Quaternion(-XYZ, W);

        public Quaternion(Vector3 xyz, float w)
        {
            XYZ = xyz;
            W = w;
        }

        public Quaternion(float x, float y, float z, float w)
        {
            XYZ = new Vector3(x, y, z);
            W = w;
        }

        public static Quaternion operator +(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q1.XYZ + q2.XYZ, q1.W + q2.W);
        }

        public static Quaternion operator -(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q1.XYZ - q2.XYZ, q1.W - q2.W);
        }

        public static Quaternion operator *(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q2.W * q1.XYZ + q1.W * q2.XYZ + Vector3.Cross(q1.XYZ, q2.XYZ), q1.W * q2.W - Vector3.Dot(q1.XYZ, q2.XYZ));
        }

        public static Quaternion operator *(Quaternion q, float s)
        {
            return new Quaternion(q.XYZ * s, q.W);
        }

        public override string ToString() => string.Format("V: ({0}, {1}, {2}), W: {3}", X, Y, Z, W);
    }
}
