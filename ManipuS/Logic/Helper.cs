using System;
using System.Collections.Generic;
using OpenTK;

public class Point
{
    public double x, y, z;

    public Point(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double Distance
    {
        get { return Math.Sqrt(x * x + y * y + z * z); }
    }

    public double DistanceTo(Point p)
    {
        return Math.Sqrt(Math.Pow(p.x - x, 2) + Math.Pow(p.y - y, 2) + Math.Pow(p.z - z, 2));
    }

    public static Point Zero
    {
        get
        {
            return new Point(0, 0, 0);
        }
    }

    public static Point operator +(Point p1, Point p2) => new Point(p2.x + p1.x, p2.y + p1.y, p2.z + p1.z);
    public static Point operator -(Point p1, Point p2) => new Point(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    public static Point operator +(Point p, Vector v) => new Point(p.x + v.x, p.y + v.y, p.z + v.z);
    public static Point operator -(Point p, Vector v) => new Point(p.x - v.x, p.y - v.y, p.z - v.z);
    public static Point operator *(Point p, double s) => new Point(p.x * s, p.y * s, p.z * s);
    public static Point operator /(Point p, double s) => new Point(p.x / s, p.y / s, p.z / s);
    public static Point operator *(Matrix m, Point p) =>
        new Point
        (
            m[0, 0] * p.x + m[0, 1] * p.y + m[0, 2] * p.z,
            m[1, 0] * p.x + m[1, 1] * p.y + m[1, 2] * p.z,
            m[2, 0] * p.x + m[2, 1] * p.y + m[2, 2] * p.z
        );
}

public class Vector
{
    public double x, y, z;

    public Vector(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector(Point p)
    {
        x = p.x;
        y = p.y;
        z = p.z;
    }

    public Vector(Point p1, Point p2)
    {
        x = p2.x - p1.x;
        y = p2.y - p1.y;
        z = p2.z - p1.z;
    }

    public double Length
    {
        get
        {
            return Math.Sqrt(x * x + y * y + z * z);
        }
    }

    public double Angle
    {
        get
        {
            return Math.Atan2(y, x) > 0 ? Math.Atan2(y, x) : Math.Atan2(y, x) + 2 * Math.PI;
        }
        set
        {
            var len = Length;
            x = Math.Cos(value) * len;
            y = Math.Sin(value) * len;
        }
    }

    public Vector Normalized
    {
        get
        {
            return new Vector(x / Length, y / Length, z / Length);
        }
    }

    public static Vector Zero
    {
        get
        {
            return new Vector(0, 0, 0);
        }
    }

    public static double AngleBetween(Vector v1, Vector v2)
    {
        return Math.Acos((v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / (v1.Length * v2.Length));
    }

    public static Vector operator +(Vector v1, Vector v2) => new Vector(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
    public static Vector operator -(Vector v1, Vector v2) => new Vector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
    public static Vector operator -(Vector v) => new Vector(-v.x, -v.y, -v.z);
    public static Vector operator *(Vector v, double s) => new Vector(v.x * s, v.y * s, v.z * s);
    public static Vector operator *(Matrix m, Vector v) =>
        new Vector
        (
            m[0, 0] * v.x + m[0, 1] * v.y + m[0, 2] * v.z,
            m[1, 0] * v.x + m[1, 1] * v.y + m[1, 2] * v.z,
            m[2, 0] * v.x + m[2, 1] * v.y + m[2, 2] * v.z
        );
}

public class VectorN  // TODO: merge with Vector class, so that it represents an arbitrary vector of dimensionality n
{
    public double[] Data;

    public VectorN(int size)
    {
        Data = new double[size];
    }

    public VectorN(double[] data)
    {
        Data = data;
    }

    public static VectorN operator *(VectorN v, double s)
    {
        for (int i = 0; i < v.Data.Length; i++)
        {
            v.Data[i] *= s;
        }

        return v;
    }

    public static VectorN operator +(VectorN v1, VectorN v2)
    {
        double[] data = new double[v1.Data.Length];
        for (int i = 0; i < v1.Data.Length; i++)
        {
            data[i] = v1[i] + v2[i];
        }

        return new VectorN(data);
    }
    public static VectorN operator *(Matrix m, VectorN v)
    {
        double[] data = new double[m.Data.GetLength(0)];
        for (int i = 0; i < m.Data.GetLength(0); i++)
        {
            for (int j = 0; j < m.Data.GetLength(1); j++)
            {
                data[i] += m[i, j] * v[j];
            }
        }

        return new VectorN(data);
    }

    public double this[int i]
    {
        get
        {
            return Data[i];
        }
    }
}

public class Matrix  // TODO: this class should not be deleted, but instead only used for non-uniform matrices (i.e. of an arbitrary size); or else mirror all OpenTK matrices into this class
                     // TODO: check why this class is much slower than OpenTK's one (note: GetLength is O(1), so it should not affect anything)
{
    public double[,] Data;

    public Matrix(double[,] Data)
    {
        this.Data = Data;
    }

    public static Matrix Identity(int size)
    {
        double[,] mat = new double[size, size];
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                if (i == j)
                    mat[i, j] = 1;
                else
                    mat[i, j] = 0;
            }
        }

        return new Matrix(mat);
    }

    /*public static Matrix TranslateD(Vector3d axis)
    {
        return new Matrix(new double[4, 4]
        {
            { 1, 0, 0, axis.X },
            { 0, 1, 0, axis.Y },
            { 0, 0, 1, axis.Z },
            { 0, 0, 0, 1 }
        });
    }

    public static Matrix RotateXD(double angle)
    {
        return new Matrix(new double[4, 4]
        {
            { 1, 0, 0, 0 },
            { 0, Math.Cos(angle), -Math.Sin(angle), 0 },
            { 0, Math.Sin(angle), Math.Cos(angle), 0 },
            { 0, 0, 0, 1 }
        });
    }

    public static Matrix RotateYD(double angle)
    {
        return new Matrix(new double[4, 4]
        {
            { Math.Cos(angle), 0, -Math.Sin(angle), 0 },
            { 0, 1, 0, 0 },
            { Math.Sin(angle), 0, Math.Cos(angle), 0 },
            { 0, 0, 0, 1 }
        });
    }*/

    public static Matrix Transpose(Matrix mat)
    {
        var r = mat.Data.GetLength(0);
        var c = mat.Data.GetLength(1);
        double[,] data = new double[c, r];
        for (int i = 0; i < r; i++)
        {
            for (int j = 0; j < c; j++)
            {
                data[j, i] = mat[i, j];
            }
        }

        return new Matrix(data);
    }

    public static Matrix4 Translate(Vector3 axis)
    {
        return new Matrix4
        (
            new Vector4(1, 0, 0, axis.X),
            new Vector4(0, 1, 0, axis.Y),
            new Vector4(0, 0, 1, axis.Z),
            new Vector4(0, 0, 0, 1)
        );
    }

    public static Matrix4 RotateX(float angle)
    {
        return new Matrix4
        (
            new Vector4(1, 0, 0, 0),
            new Vector4(0, (float)Math.Cos(angle), (float)-Math.Sin(angle), 0),
            new Vector4(0, (float)Math.Sin(angle), (float)Math.Cos(angle), 0),
            new Vector4(0, 0, 0, 1)
        );
    }

    public static Matrix4 RotateY(float angle)
    {
        return new Matrix4
        (
            new Vector4((float)Math.Cos(angle), 0, (float)-Math.Sin(angle), 0),
            new Vector4(0, 1, 0, 0),
            new Vector4((float)Math.Sin(angle), 0, (float)Math.Cos(angle), 0),
            new Vector4(0, 0, 0, 1)
        );
    }

    public static Matrix4 RotateZ(float angle)
    {
        return new Matrix4
        (
            new Vector4((float)Math.Cos(angle), (float)-Math.Sin(angle), 0, 0),
            new Vector4((float)Math.Sin(angle), (float)Math.Cos(angle), 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );
    }

    public static Matrix4 RotateCustomAnother(Vector4 point, Vector4 dir, float angle)  // TODO: optimize; too many complex operations
    {
        float x = point.X,
              y = point.Y,
              z = point.Z;

        var n = dir.Normalized();
        float a = n.X,
              b = n.Y,
              c = n.Z,
              d = (float)Math.Sqrt(b * b + c * c);

        float cos = (float)Math.Cos(angle),
              sin = (float)Math.Sin(angle);

        Matrix4 T = new Matrix4
        (
            new Vector4(1, 0, 0, -x),
            new Vector4(0, 1, 0, -y),
            new Vector4(0, 0, 1, -z),
            new Vector4(0, 0, 0, 1)
        );

        Matrix4 Rx = new Matrix4
        (
            new Vector4(1, 0, 0, 0),
            new Vector4(0, c / d, -b / d, 0),
            new Vector4(0, b / d, c / d, 0),
            new Vector4(0, 0, 0, 1)
        );

        Matrix4 Ry = new Matrix4
        (
            new Vector4(d, 0, -a, 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(a, 0, d, 0),
            new Vector4(0, 0, 0, 1)
        );

        Matrix4 Rz = new Matrix4
        (
            new Vector4(cos, -sin, 0, 0),
            new Vector4(sin, cos, 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );

        return T.Inverted() * Rx.Inverted() * Ry.Inverted() * Rz * Ry * Rx * T;
        //return T * Rx * Ry * Rz * Ry.Inverted() * Rx.Inverted() * T.Inverted();
    }

    public static Matrix Rotation(int axis, double angle)
    {
        double[,] mat = new double[3, 3];
        switch (axis)
        {
            case 0:
                mat = new double[3, 3]
                {
                        { 1, 0, 0 },
                        { 0, Math.Cos(angle), -Math.Sin(angle) },
                        { 0, Math.Sin(angle), Math.Cos(angle) }
                };
                break;
            case 1:
                mat = new double[3, 3]
                {
                        { Math.Cos(angle), 0, -Math.Sin(angle) },
                        { 0, 1, 0 },
                        { Math.Sin(angle), 0, Math.Cos(angle) }
                };
                break;
            case 2:
                mat = new double[3, 3]
                {
                        { Math.Cos(angle), -Math.Sin(angle), 0 },
                        { Math.Sin(angle), Math.Cos(angle), 0 },
                        { 0, 0, 1 }
                };
                break;
        }

        return new Matrix(mat);
    }

    public double this[int r, int c]
    {
        get
        {
            if (r < 0 || r > Data.GetLength(0))
                throw new Exception("Row index out of range!");
            else if (c < 0 || c > Data.GetLength(1))
                throw new Exception("Column index out of range!");
            else
                return Data[r, c];
        }
        set
        {
            Data[r, c] = value;
        }
    }

    public static Matrix operator +(Matrix m1, Matrix m2)
    {
        int r1 = m1.Data.GetLength(0), c1 = m1.Data.GetLength(1),
            r2 = m2.Data.GetLength(0), c2 = m2.Data.GetLength(1);

        double[,] mat;
        if (c1 != c2 || r1 != r2)
            throw new Exception("Matrices dimensions mismatch!");
        else
        {
            mat = new double[r1, c1];
            for (int i = 0; i < r1; i++)
            {
                for (int j = 0; j < c1; j++)
                {
                    mat[i, j] = m1[i, j] + m2[i, j];
                }
            }
        }

        return new Matrix(mat);
    }

    public static Matrix operator *(Matrix m1, Matrix m2)
    {
        int r1 = m1.Data.GetLength(0), c1 = m1.Data.GetLength(1),
            r2 = m2.Data.GetLength(0), c2 = m2.Data.GetLength(1);

        double[,] mat;
        if (c1 != r2)
            throw new Exception("Matrices dimensions mismatch!");
        else
        {
            mat = new double[r1, c2];
            for (int i = 0; i < r1; i++)
            {
                for (int j = 0; j < c2; j++)
                {
                    double t = 0;
                    for (int k = 0; k < c1; k++)
                    {
                        t += m1[i, k] * m2[k, j];
                    }
                    mat[i, j] = t;
                }
            }
        }

        return new Matrix(mat);
    }
}

namespace Logic
{
    public class Segment
    {
        public Point p1, p2;
        public double minX, maxX, minY, maxY;

        public Segment(Point point1, Point point2)
        {
            p1 = point1;
            p2 = point2;

            minX = Math.Min(p1.x, p2.x);
            maxX = Math.Max(p1.x, p2.x);
            minY = Math.Min(p1.y, p2.y);
            maxY = Math.Max(p1.y, p2.y);
        }

        public bool Contains(Point p)
        {
            if (p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY)
                return true;
            else
                return false;
        }

        public Point[] Discretize(int segments)
        {
            Point[] pos = new Point[segments + 1];
            for (int j = 0; j < segments + 1; j++)
            {
                pos[j] = new Point
                (
                    p1.x + j * (p2.x - p1.x) / segments,
                    p1.y + j * (p2.y - p1.y) / segments,
                    p1.z + j * (p2.z - p1.z) / segments
                );
            }
            return pos;
        }
    }

    public class Attractor  // TODO: maybe struct would be better?
    {
        public Point Center;
        public double Weight;
        public Point[] Area;
        public double Radius;
        public double InliersCount;

        public Attractor(Point center, double weight, Point[] area, double radius)
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
        public static double[] ToRad(double[] degrees)
        {
            return Array.ConvertAll(degrees, (t) => { return t * Math.PI / 180; });
        }

        public static float[] ToRad(float[] degrees)
        {
            return Array.ConvertAll(degrees, (t) => { return (float)(t * Math.PI / 180); });
        }

        public static T[] CopyArray<T>(T[] source)
        {
            T[] destination = new T[source.Length];
            Array.Copy(source, destination, source.Length);
            return destination;
        }

        public static T[,] CopyArray<T>(T[,] source)
        {
            T[,] destination = new T[source.GetLength(0), source.GetLength(1)];
            Array.Copy(source, destination, source.Length);
            return destination;
        }

        public static double BoxMullerTransform(Random rng, double mu, double sigma)
        {
            double phi = 0, r = 0;
            while (phi == 0)
                phi = rng.NextDouble();
            while (r == 0)
                r = rng.NextDouble();

            double z = Math.Cos(2 * Math.PI * phi) * Math.Sqrt(-2 * Math.Log(r));
            return mu + sigma * z;
        }
    }

    public static class Primitives
    {
        public static Point[] Sphere(double r, Point c, int points_num, Random rng)
        {
            Point[] sphere = new Point[points_num];
            double x, y_pos, y, z_pos, z;
            for (int i = 0; i < points_num; i++)
            {
                x = -r + rng.NextDouble() * 2 * r;
                y_pos = Math.Sqrt(r * r - x * x);
                y = -y_pos + rng.NextDouble() * 2 * y_pos;
                z_pos = Math.Sqrt(r * r - x * x - y * y);
                z = rng.Next(0, 2) == 0 ? -z_pos : z_pos;

                sphere[i] = new Point(x, y, z) + c;
            }

            return sphere;
        }
    }
}
