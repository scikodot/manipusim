using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Logic
{
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

        /*public Vector Ortho
        {
            get
            {
                return new Vector(y, -x);
            }
        }*/

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

        /*public Segment(Point point1, Point point2, Point offsetRef, double offset)
        {
            Vector ortho = new Vector(point2, point1).Ortho;
            double A = ortho.x, B = ortho.y, C = -ortho.x * point1.x - ortho.y * point1.y;
            double d = (A * offsetRef.x + B * offsetRef.y + C) / Math.Sqrt(A * A + B * B);

            p1 = point1 + (d > 0 ? -ortho : ortho).Normalized * offset;
            p2 = point2 + (d > 0 ? -ortho : ortho).Normalized * offset;

            minX = Math.Min(p1.x, p2.x);
            maxX = Math.Max(p1.x, p2.x);
            minY = Math.Min(p1.y, p2.y);
            maxY = Math.Max(p1.y, p2.y);
        }*/

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

    public class Matrix
    {
        double[,] Data;

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

    public class Tree
    {
        public class Node
        {
            public Node Parent;
            public List<Node> Childs;
            public int Layer;
            public Point p;
            public double[] q;

            public Node(Node parent, Point p, double[] q)
            {
                Parent = parent;
                Childs = new List<Node>();
                if (parent == null)
                    Layer = 0;
                else
                    Layer = parent.Layer + 1;
                this.p = p;
                this.q = Misc.CopyArray(q);
            }
        }

        public List<List<Node>> Layers;
        public List<Node> Buffer;
        public int Count, LayersAdded;

        public Tree(Node root)
        {
            Layers = new List<List<Node>>
            {
                new List<Node>()
            };
            Buffer = new List<Node>();

            Layers[0].Add(root);
            Count = 1;
            LayersAdded = 0;
        }

        public Node Root
        {
            get { return Layers[0][0]; }
        }

        public void AddLayer()
        {
            Layers.Add(new List<Node>());
            LayersAdded++;
        }

        public void RemoveLayer()
        {
            int LayerCount = Layers[Layers.Count - 1].Count;
            Layers.RemoveAt(Layers.Count - 1);
            Count -= LayerCount;
        }

        public void RemoveNode(Node n)
        {
            Layers[n.Layer].Remove(n);
            Count--;
        }

        public void AddNode(Node n)
        {
            if (n.Layer == Layers.Count)
                AddLayer();

            n.Parent.Childs.Add(n);
            //if (n.Parent.Childs.Count > 1)
            //{
            //    Rectify(n.Parent);
            //}
            
            Layers[n.Layer].Add(n);
            Count++;
        }

        /*public void Concat(Tree tree, Tuple<int, int> coords)
        {
            for (int i = 1; i < tree.Layers.Count; i++)
            {
                int parent_layer = coords.Item1;
                if (i == 1)
                {
                    foreach (var node in tree.Layers[i])
                    {
                        node.Layer = parent_layer + 1;
                        node.Parent = Layers[coords.Item1][coords.Item2];
                        AddNode(node);
                    }
                }
                else
                {
                    foreach (var node in tree.Layers[i])
                    {
                        node.Layer = parent_layer + i;
                        AddNode(node);
                    }
                }
            }
        }*/

        public Node Min(Point p)
        {
            Node min_node = null;
            double min = double.PositiveInfinity;
            foreach (var layer in Layers)
            {
                foreach (var node in layer)
                {
                    double curr = p.DistanceTo(node.p);
                    if (curr < min)
                    {
                        min = curr;
                        min_node = node;
                    }
                }
            }
            return min_node;
        }

        /*public Node MinWeight()
        {
            Node min_node = null;
            double min = double.PositiveInfinity;
            foreach (var layer in Layers)
            {
                foreach (var node in layer)
                {
                    double curr = node.Weight;
                    if (curr < min)
                    {
                        min = curr;
                        min_node = node;
                    }
                }
            }
            return min_node;
        }*/

        public void Rectify(Node start)
        {
            Node node_curr = start.Parent;
            List<Node> nodes = new List<Node>();
            while (node_curr != null && node_curr.Parent != null && node_curr.Childs.Count == 1)
            {
                nodes.Add(node_curr);
                node_curr = node_curr.Parent;
            }

            if (nodes.Count > 0)
            {
                foreach (var node in nodes)
                {
                    RemoveNode(node);
                }
                start.Parent = node_curr;
            }
        }

        public void RectifyWhole()
        {
            for (int i = Layers.Count - 1; i >= 0; i--)
            {
                for (int j = 0; j < Layers[i].Count; j++)
                {
                    Rectify(Layers[i][j]);
                }
            }
        }

        public static Node[] Discretize(Node start, Node end, int pointNum)
        {
            Segment seg = new Segment(start.p, end.p);
            Point[] points = new Point[pointNum];
            Array.Copy(seg.Discretize(pointNum + 1), 1, points, 0, pointNum);

            Node parent, child;
            if (start.Layer > end.Layer)
            {
                parent = end;
                child = start;
            }
            else
            {
                parent = start;
                child = end;
            }

            double[][] configs = new double[pointNum][];
            for (int i = 0; i < pointNum; i++)
            {
                double[] config = new double[start.q.Length];
                for (int j = 0; j < start.q.Length; j++)
                {
                    config[j] = start.q[j] + (i + 1) * (end.q[j] - start.q[j]) / (pointNum + 1);
                }
                configs[i] = config;
            }

            Node[] nodes = new Node[pointNum];
            for (int i = 0; i < pointNum; i++)
            {
                nodes[i] = new Node(null, points[i], configs[i]);
            }
            return nodes;
        }
    }

    /*public class AssociativeTable
    {
        public Dictionary<double, double[]> Table;

        public AssociativeTable(Manipulator Agent, int entries)
        {
            Table = new Dictionary<double, double[]>();
            double[] q_init = new double[IKP.ParamNum];
            Array.Copy(Agent.q, q_init, IKP.ParamNum);

            Random rng = new Random();
            IKP Solver = new IKP(0.02, Agent.Links.Length, 10, 0.2, 50);
            for (int i = 0; i < 3600; i++)
            {
                double d = 0.04, a = i;  //a = Math.Round(-Math.PI + 2 * Math.PI * rng.NextDouble(), 2);
                Vector v = new Vector(d * Math.Cos(a / 10), d * Math.Sin(a / 10));

                var res = Solver.Execute(Agent.GripperPos + v);
                Array.Copy(q_init, Agent.q, IKP.ParamNum);
                while (!res.Item1)
                {
                    res = Solver.Execute(Agent.GripperPos + v);
                    Array.Copy(q_init, Agent.q, IKP.ParamNum);
                }

                Table.Add(a, res.Item3);
            }
        }
    }*/

    public class Attractor
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

        /*public static List<float> ToRad(List<float> degrees)
        {
            return degrees.ConvertAll((t) => { return (float)(t * Math.PI / 180); });
        }*/

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

            double z = Math.Cos(Math.PI * phi) * Math.Sqrt(-2 * Math.Log(r));
            return mu + sigma * z;
        }
    }

    public static class Primitives
    {
        public static Random Rng = new Random();

        public static Point[] Sphere(double r, Point c, int points_num)
        {
            Point[] sphere = new Point[points_num];
            double x, y_pos, y, z_pos, z;
            for (int i = 0; i < points_num; i++)
            {
                x = -r + Rng.NextDouble() * 2 * r;
                y_pos = Math.Sqrt(r * r - x * x);
                y = -y_pos + Rng.NextDouble() * 2 * y_pos;
                z_pos = Math.Sqrt(r * r - x * x - y * y);
                z = Rng.Next(0, 2) == 0 ? -z_pos : z_pos;

                sphere[i] = new Point(x, y, z) + c;
            }

            return sphere;
        }
    }
}
