using System;

namespace Logic
{
    public struct Vector4
    {
        public float X { get; }
        public float Y { get; }
        public float Z { get; }
        public float W { get; }

        public static Vector4 Null => new Vector4(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);

        public static Vector4 Zero => new Vector4(0, 0, 0, 0);

        public static Vector4 UnitX => new Vector4(1, 0, 0, 0);

        public static Vector4 UnitY => new Vector4(0, 1, 0, 0);

        public static Vector4 UnitZ => new Vector4(0, 0, 1, 0);

        public static Vector4 UnitW => new Vector4(0, 0, 0, 1);

        public static Vector4 One => new Vector4(1, 1, 1, 1);

        public Vector3 SubVector3 => new Vector3(X, Y, Z);

        public float Length => (float)Math.Sqrt(X * X + Y * Y + Z * Z + W * W);

        public float LengthSquared => X * X + Y * Y + Z * Z + W * W;

        public Vector4 Normalized => this / Length;

        public Vector4(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public Vector4(Vector3 v, float w)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
            W = w;
        }

        public static float Dot(Vector4 v1, Vector4 v2)
        {
            return v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z + v1.W * v2.W;
        }

        public static Vector4 operator +(Vector4 v1, Vector4 v2)
        {
            return new Vector4(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z, v1.W + v2.W);
        }

        public static Vector4 operator -(Vector4 v)
        {
            return new Vector4(-v.X, -v.Y, -v.Z, -v.W);
        }

        public static Vector4 operator -(Vector4 v1, Vector4 v2)
        {
            return new Vector4(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z, v1.W - v2.W);
        }

        public static Vector4 operator *(Matrix4 m, Vector4 v)
        {
            return new Vector4(
                Dot(m.Row0, v),
                Dot(m.Row1, v),
                Dot(m.Row2, v),
                Dot(m.Row3, v)
            );
        }

        public static Vector4 operator *(Vector4 v, float s)
        {
            return new Vector4(v.X * s, v.Y * s, v.Z * s, v.W * s);
        }

        public static Vector4 operator *(float s, Vector4 v)
        {
            return new Vector4(v.X * s, v.Y * s, v.Z * s, v.W * s);
        }

        public static Vector4 operator /(Vector4 v, float s)
        {
            return new Vector4(v.X / s, v.Y / s, v.Z / s, v.W / s);
        }

        public static bool operator ==(Vector4 v1, Vector4 v2)
        {
            return v1.Equals(v2);
        }

        public static bool operator !=(Vector4 v1, Vector4 v2)
        {
            return !v1.Equals(v2);
        }

        public static implicit operator OpenTK.Vector4(Vector4 v) => new OpenTK.Vector4(v.X, v.Y, v.Z, v.W);
        public static implicit operator Vector4(OpenTK.Vector4 v) => new Vector4(v.X, v.Y, v.Z, v.W);
        //public static explicit operator OpenTK.Vector4(Vector3 v) => new OpenTK.Vector4(v.X, v.Y, v.Z, 1);
        //public static explicit operator Vector3(OpenTK.Vector4 v) => new Vector3(v.X, v.Y, v.Z);

        public override bool Equals(object obj)
        {
            if (!(obj is Vector4))
                return false;

            return Equals((Vector4)obj);
        }

        public bool Equals(Vector4 other)
        {
            return
                X == other.X &&
                Y == other.Y &&
                Z == other.Z &&
                W == other.W;
        }

        private static readonly string ListSeparator = System.Globalization.CultureInfo.CurrentCulture.TextInfo.ListSeparator;
        public override string ToString() => string.Format("({0:0.###}{4} {1:0.###}{4} {2:0.###}{4} {3:0.###})", X, Y, Z, W, ListSeparator);
    }
}