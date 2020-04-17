using System;
using System.Numerics;

namespace Logic
{
    public struct Matrix4
    {
        public Vector4 Row0 { get; }
        public Vector4 Row1 { get; }
        public Vector4 Row2 { get; }
        public Vector4 Row3 { get; }
        private readonly bool transposed;

        public Vector4 Column0 => new Vector4(Row0.X, Row1.X, Row2.X, Row3.X);
        public Vector4 Column1 => new Vector4(Row0.Y, Row1.Y, Row2.Y, Row3.Y);
        public Vector4 Column2 => new Vector4(Row0.Z, Row1.Z, Row2.Z, Row3.Z);
        public Vector4 Column3 => new Vector4(Row0.W, Row1.W, Row2.W, Row3.W);

        public static Matrix4 Identity => new Matrix4(Vector4.UnitX, Vector4.UnitY, Vector4.UnitZ, Vector4.UnitW);

        //public Matrix3 Rotation => transposed ? new Matrix3(Column0.XYZ, Column1.XYZ, Column2.XYZ) : new Matrix3(Row0.XYZ, Row1.XYZ, Row2.XYZ);

        //public Vector3 Translation => transposed ? Row3.XYZ : Column3.XYZ;

        public Matrix4(Vector4 row0, Vector4 row1, Vector4 row2, Vector4 row3, bool transpose = false)  // TODO: probably make flag for transpose instead of bool?
        {
            Row0 = row0;
            Row1 = row1;
            Row2 = row2;
            Row3 = row3;
            transposed = transpose;
        }

        public Matrix4(float m00, float m01, float m02, float m03,
                       float m10, float m11, float m12, float m13,
                       float m20, float m21, float m22, float m23,
                       float m30, float m31, float m32, float m33, bool transpose = false)
        {
            Row0 = new Vector4(m00, m01, m02, m03);
            Row1 = new Vector4(m10, m11, m12, m13);
            Row2 = new Vector4(m20, m21, m22, m23);
            Row3 = new Vector4(m30, m31, m32, m33);
            transposed = false;
        }

        public static Matrix4 CreateTranslation(Vector3 offset, bool transpose = false)
        {
            if (transpose)
                return new Matrix4(
                    new Vector4(1, 0, 0, 0),
                    new Vector4(0, 1, 0, 0),
                    new Vector4(0, 0, 1, 0),
                    new Vector4(offset, 1),
                    transpose
                );
            else
                return new Matrix4(
                    new Vector4(1, 0, 0, offset.X),
                    new Vector4(0, 1, 0, offset.Y),
                    new Vector4(0, 0, 1, offset.Z),
                    new Vector4(0, 0, 0, 1)
                );
        }

        public static Matrix4 CreateRotationX(float angle, bool transpose = false)
        {
            var cos = (float)Math.Cos(angle);
            var sin = (float)Math.Sin(angle);

            if (transpose)
                return new Matrix4(
                    new Vector4(1, 0, 0, 0),
                    new Vector4(0, cos, sin, 0),
                    new Vector4(0, -sin, cos, 0),
                    new Vector4(0, 0, 0, 1),
                    transpose
                );
            else
                return new Matrix4(
                    new Vector4(1, 0, 0, 0),
                    new Vector4(0, cos, -sin, 0),
                    new Vector4(0, sin, cos, 0),
                    new Vector4(0, 0, 0, 1)
                );
        }

        public static Matrix4 CreateRotationY(float angle, bool transpose = false)
        {
            var cos = (float)Math.Cos(angle);
            var sin = (float)Math.Sin(angle);

            if (transpose)
                return new Matrix4(
                    new Vector4(cos, 0, sin, 0),
                    new Vector4(0, 1, 0, 0),
                    new Vector4(-sin, 0, cos, 0),
                    new Vector4(0, 0, 0, 1),
                    transpose
                );
            else
                return new Matrix4(
                    new Vector4(cos, 0, -sin, 0),
                    new Vector4(0, 1, 0, 0),
                    new Vector4(sin, 0, cos, 0),
                    new Vector4(0, 0, 0, 1)
                );
        }

        public static Matrix4 CreateRotationZ(float angle, bool transpose = false)
        {
            var cos = (float)Math.Cos(angle);
            var sin = (float)Math.Sin(angle);

            if (transpose)
                return new Matrix4(
                    new Vector4(cos, sin, 0, 0),
                    new Vector4(-sin, cos, 0, 0),
                    new Vector4(0, 0, 1, 0),
                    new Vector4(0, 0, 0, 1),
                    transpose
                );
            else
                return new Matrix4(
                    new Vector4(cos, -sin, 0, 0),
                    new Vector4(sin, cos, 0, 0),
                    new Vector4(0, 0, 1, 0),
                    new Vector4(0, 0, 0, 1)
                );
        }

        public static Matrix4 operator +(Matrix4 m1, Matrix4 m2)
        {
            return new Matrix4(
                m1.Row0 + m2.Row0,
                m1.Row1 + m2.Row1,
                m1.Row2 + m2.Row2,
                m1.Row3 + m2.Row3
            );
        }

        public static Matrix4 operator -(Matrix4 m1, Matrix4 m2)
        {
            return new Matrix4(
                m1.Row0 - m2.Row0,
                m1.Row1 - m2.Row1,
                m1.Row2 - m2.Row2,
                m1.Row3 - m2.Row3
            );
        }

        public static Matrix4 operator *(Matrix4 m1, Matrix4 m2)
        {
            Vector4
                row0 = m1.Row0,
                row1 = m1.Row1,
                row2 = m1.Row2,
                row3 = m1.Row3;
            Vector4
                column0 = m2.Column0,
                column1 = m2.Column1,
                column2 = m2.Column2,
                column3 = m2.Column3;

            float
                dot00 = Vector4.Dot(row0, column0), dot01 = Vector4.Dot(row0, column1), dot02 = Vector4.Dot(row0, column2), dot03 = Vector4.Dot(row0, column3),
                dot10 = Vector4.Dot(row1, column0), dot11 = Vector4.Dot(row1, column1), dot12 = Vector4.Dot(row1, column2), dot13 = Vector4.Dot(row1, column3),
                dot20 = Vector4.Dot(row2, column0), dot21 = Vector4.Dot(row2, column1), dot22 = Vector4.Dot(row2, column2), dot23 = Vector4.Dot(row2, column3),
                dot30 = Vector4.Dot(row3, column0), dot31 = Vector4.Dot(row3, column1), dot32 = Vector4.Dot(row3, column2), dot33 = Vector4.Dot(row3, column3);

            return new Matrix4(
                new Vector4(dot00, dot01, dot02, dot03),
                new Vector4(dot10, dot11, dot12, dot13),
                new Vector4(dot20, dot21, dot22, dot23),
                new Vector4(dot30, dot31, dot32, dot33)
            );
        }

        public static Matrix4 operator *(Matrix4 m, float s)
        {
            return new Matrix4(m.Row0 * s, m.Row1 * s, m.Row2 * s, m.Row3 * s);
        }

        public static Matrix4 operator *(float s, Matrix4 m)
        {
            return new Matrix4(m.Row0 * s, m.Row1 * s, m.Row2 * s, m.Row3 * s);
        }

        public static Matrix4 operator /(Matrix4 m, float s)
        {
            return new Matrix4(m.Row0 / s, m.Row1 / s, m.Row2 / s, m.Row3 / s);
        }

        public static implicit operator OpenTK.Matrix4(Matrix4 m) => new OpenTK.Matrix4(m.Row0.X, m.Row0.Y, m.Row0.Z, m.Row0.W,
                                                                                        m.Row1.X, m.Row1.Y, m.Row1.Z, m.Row1.W,
                                                                                        m.Row2.X, m.Row2.Y, m.Row2.Z, m.Row2.W,
                                                                                        m.Row3.X, m.Row3.Y, m.Row3.Z, m.Row3.W);
        public static implicit operator Matrix4(OpenTK.Matrix4 m) => new Matrix4(m.Row0.X, m.Row0.Y, m.Row0.Z, m.Row0.W,
                                                                                 m.Row1.X, m.Row1.Y, m.Row1.Z, m.Row1.W,
                                                                                 m.Row2.X, m.Row2.Y, m.Row2.Z, m.Row2.W,
                                                                                 m.Row3.X, m.Row3.Y, m.Row3.Z, m.Row3.W);

        public override string ToString() => $"{Row0}\n{Row1}\n{Row2}\n{Row3}";
    }
}
