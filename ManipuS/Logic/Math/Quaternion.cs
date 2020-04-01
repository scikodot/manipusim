using System;

namespace Logic
{
    public struct Quaternion
    {
        public Vector4 XYZW { get; }

        public float X => XYZW.X;
        public float Y => XYZW.Y;
        public float Z => XYZW.Z;
        public float W => XYZW.W;

        public Vector3 XYZ => XYZW.XYZ;

        public static Quaternion Null => new Quaternion(0, 0, 0, 0);

        public static Quaternion Zero => new Quaternion(0, 0, 0, 1);

        public float Length => XYZW.Length;

        public float LengthSquared => XYZW.LengthSquared;

        public Quaternion Normalized => new Quaternion(XYZW.Normalized);

        public Quaternion Conjugate => new Quaternion(-XYZ, W);

        public Quaternion(Vector4 xyzw)
        {
            XYZW = xyzw;
        }

        public Quaternion(Vector3 xyz, float w)
        {
            XYZW = new Vector4(xyz, w);
        }

        public Quaternion(float x, float y, float z, float w)
        {
            XYZW = new Vector4(x, y, z, w);
        }

        public Vector3 Transform(Vector3 v)
        {
            Vector3 uv = Vector3.Cross(XYZ, v);
            uv += uv;
            return v + W * uv + Vector3.Cross(XYZ, uv);
        }

        public static Quaternion operator +(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q1.XYZW + q2.XYZW);
        }

        public static Quaternion operator -(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q1.XYZW - q2.XYZW);
        }

        public static Quaternion operator *(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q2.W * q1.XYZ + q1.W * q2.XYZ + Vector3.Cross(q1.XYZ, q2.XYZ), q1.W * q2.W - Vector3.Dot(q1.XYZ, q2.XYZ));
        }

        public static Quaternion operator *(Quaternion q, float s)
        {
            return new Quaternion(q.XYZW * s);
        }
        public static Quaternion operator *(float s, Quaternion q)
        {
            return new Quaternion(q.XYZW * s);
        }

        public static Quaternion operator /(Quaternion q, float s)
        {
            return new Quaternion(q.XYZW / s);
        }

        public override string ToString() => string.Format("V: ({0}, {1}, {2}), W: {3}", X, Y, Z, W);
    }
}
