using System;

namespace Logic
{
    public struct Vector3
    {
        public float X { get; }
        public float Y { get; }
        public float Z { get; }

        public static Vector3 Zero => new Vector3(0, 0, 0);

        public static Vector3 UnitX => new Vector3(1, 0, 0);

        public static Vector3 UnitY => new Vector3(0, 1, 0);

        public static Vector3 UnitZ => new Vector3(0, 0, 1);

        public static Vector3 One => new Vector3(1, 1, 1);

        public float Length => (float)Math.Sqrt(X * X + Y * Y + Z * Z);

        public float LengthSquared => X * X + Y * Y + Z * Z;

        public Vector3 Normalized => this / Length;

        public Vector3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public Vector3(Vector3 v1, Vector3 v2)
        {
            this = v2 - v1;
        }

        public float DistanceTo(Vector3 v)
        {
            return (this - v).Length;
        }

        public static float Dot(Vector3 v1, Vector3 v2)
        {
            return v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z;
        }

        public static Vector3 Cross(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1.Y * v2.Z - v2.Y * v1.Z, -v1.X * v2.Z + v2.X * v1.Z, v1.X * v2.Y - v2.X * v1.Y);
        }

        public static Vector3 operator +(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
        }

        public static Vector3 operator -(Vector3 v)
        {
            return new Vector3(-v.X, -v.Y, -v.Z);
        }

        public static Vector3 operator -(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
        }

        public static Vector3 operator *(Vector3 v, float s)
        {
            return new Vector3(v.X * s, v.Y * s, v.Z * s);
        }

        public static Vector3 operator *(float s, Vector3 v)
        {
            return new Vector3(v.X * s, v.Y * s, v.Z * s);
        }

        public static Vector3 operator /(Vector3 v, float s)
        {
            return new Vector3(v.X / s, v.Y / s, v.Z / s);
        }

        public static bool operator ==(Vector3 v1, Vector3 v2)
        {
            return (v1.X == v2.X && v1.Y == v2.Y && v1.Z == v2.Z) ? true : false;
        }

        public static bool operator !=(Vector3 v1, Vector3 v2)
        {
            return (v1.X == v2.X && v1.Y == v2.Y && v1.Z == v2.Z) ? false : true;
        }

        public static implicit operator OpenTK.Vector3(Vector3 v) => new OpenTK.Vector3(v.X, v.Y, v.Z);
        public static implicit operator Vector3(OpenTK.Vector3 v) => new Vector3(v.X, v.Y, v.Z);
        public static explicit operator Vector3(OpenTK.Vector4 v) => new Vector3(v.X, v.Y, v.Z);

        private static readonly string ListSeparator = System.Globalization.CultureInfo.CurrentCulture.TextInfo.ListSeparator;
        public override string ToString() => string.Format("({0:#.###}{3} {1:#.###}{3} {2:#.###})", X, Y, Z, ListSeparator);
    }
}
