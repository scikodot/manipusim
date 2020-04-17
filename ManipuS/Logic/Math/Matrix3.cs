using System;
using System.Numerics;

namespace Logic
{
    public struct Matrix3
    {
        public Vector3 Row0 { get; }
        public Vector3 Row1 { get; }
        public Vector3 Row2 { get; }

        public Vector3 Column0 => new Vector3(Row0.X, Row1.X, Row2.X);
        public Vector3 Column1 => new Vector3(Row0.Y, Row1.Y, Row2.Y);
        public Vector3 Column2 => new Vector3(Row0.Z, Row1.Z, Row2.Z);

        public static Matrix3 Identity => new Matrix3(Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ);

        public Matrix3(Vector3 row0, Vector3 row1, Vector3 row2)
        {
            Row0 = row0;
            Row1 = row1;
            Row2 = row2;
        }

        public Matrix3(float m00, float m01, float m02,
                       float m10, float m11, float m12,
                       float m20, float m21, float m22)
        {
            Row0 = new Vector3(m00, m01, m02);
            Row1 = new Vector3(m10, m11, m12);
            Row2 = new Vector3(m20, m21, m22);
        }

        public static Matrix3 CreateRotationX(float angle)
        {
            var cos = (float)Math.Cos(angle);
            var sin = (float)Math.Sin(angle);
            return new Matrix3(
                new Vector3(cos, -sin, 0),
                new Vector3(sin, cos, 0),
                new Vector3(0, 0, 1)
            );
        }

        public static Matrix3 CreateRotationY(float angle)
        {
            var cos = (float)Math.Cos(angle);
            var sin = (float)Math.Sin(angle);
            return new Matrix3(
                new Vector3(cos, 0, -sin),
                new Vector3(0, 1, 0),
                new Vector3(sin, 0, cos)
            );
        }

        public static Matrix3 CreateRotationZ(float angle)
        {
            var cos = (float)Math.Cos(angle);
            var sin = (float)Math.Sin(angle);
            return new Matrix3(
                new Vector3(1, 0, 0),
                new Vector3(0, cos, -sin),
                new Vector3(0, sin, cos)
            );
        }

        public static Matrix3 operator +(Matrix3 m1, Matrix3 m2)
        {
            return new Matrix3(
                m1.Row0 + m2.Row0,
                m1.Row1 + m2.Row1,
                m1.Row2 + m2.Row2
            );
        }

        public static Matrix3 operator -(Matrix3 m1, Matrix3 m2)
        {
            return new Matrix3(
                m1.Row0 - m2.Row0,
                m1.Row1 - m2.Row1,
                m1.Row2 - m2.Row2
            );
        }

        public static Matrix3 operator *(Matrix3 m1, Matrix3 m2)
        {
            Vector3
                row0 = m1.Row0,
                row1 = m1.Row1,
                row2 = m1.Row2;
            Vector3
                column0 = m2.Column0,
                column1 = m2.Column1,
                column2 = m2.Column2;

            float
                dot00 = Vector3.Dot(row0, column0),
                dot01 = Vector3.Dot(row0, column1),
                dot02 = Vector3.Dot(row0, column2),
                dot10 = Vector3.Dot(row1, column0),
                dot11 = Vector3.Dot(row1, column1),
                dot12 = Vector3.Dot(row1, column2),
                dot20 = Vector3.Dot(row2, column0),
                dot21 = Vector3.Dot(row2, column1),
                dot22 = Vector3.Dot(row2, column2);

            return new Matrix3(
                new Vector3(dot00, dot01, dot02),
                new Vector3(dot10, dot11, dot12),
                new Vector3(dot20, dot21, dot22)
            );
        }

        public static Matrix3 operator *(Matrix3 m, float s)
        {
            return new Matrix3(m.Row0 * s, m.Row1 * s, m.Row2 * s);
        }

        public static Matrix3 operator *(float s, Matrix3 m)
        {
            return new Matrix3(m.Row0 * s, m.Row1 * s, m.Row2 * s);
        }

        public static Matrix3 operator /(Matrix3 m, float s)
        {
            return new Matrix3(m.Row0 / s, m.Row1 / s, m.Row2 / s);
        }

        public static implicit operator OpenTK.Matrix3(Matrix3 m) => new OpenTK.Matrix3(m.Row0.X, m.Row0.Y, m.Row0.Z,
                                                                                        m.Row1.X, m.Row1.Y, m.Row1.Z,
                                                                                        m.Row2.X, m.Row2.Y, m.Row2.Z);
        public static implicit operator Matrix3(OpenTK.Matrix3 m) => new Matrix3(m.Row0.X, m.Row0.Y, m.Row0.Z,
                                                                                 m.Row1.X, m.Row1.Y, m.Row1.Z,
                                                                                 m.Row2.X, m.Row2.Y, m.Row2.Z);

        public override string ToString() => $"{Row0}\n{Row1}\n{Row2}";
    }
}
