using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK.Graphics.OpenGL4;

namespace Logic
{   
    public struct TupleDH
    {
        public Func<Manipulator, double> theta;
        public double d;
        public double alpha;
        public double r;

        public TupleDH(Func<Manipulator, double> theta, double d, double alpha, double r)
        {
            this.theta = theta;
            this.d = d;
            this.alpha = alpha;
            this.r = r;
        }
    }

    public struct ManipData
    {
        public int N, N_prev;
        public System.Numerics.Vector3 Base;
        public float[] l, q;
        public System.Numerics.Vector2[] q_ranges;
        public System.Numerics.Vector4[] DH;
        public System.Numerics.Vector3 Goal;

        public bool ShowTree;
    }

    public struct ObstData
    {
        public float r;
        public System.Numerics.Vector3 c;
        public int points_num;

        public bool ShowBounding;
    }

    public struct AlgData
    {
        public int AttrNum;

        public float Precision, StepSize;
        public int MaxTime;

        public int k;
        public float d;
    }

    public class Manipulator
    {
        public Point Base;
        public double[] l, q;
        public double[,] q_ranges;
        public TupleDH[] DH;
        public List<Matrix> RZ, TZ, RX, TX;

        public Point Goal;
        public List<Point> Path;
        public List<Point[]> Joints;
        public Tree Tree;
        public List<Attractor> Attractors;
        public List<Tree.Node> Buffer = new List<Tree.Node>();
        public Dictionary<string, bool> States;

        public Manipulator()
        {

        }

        public Manipulator(Point Base, double[] l, double[] q_init, double[,] q_ranges, TupleDH[] DH, Point Goal)
        {
            this.Base = Base;
            this.l = Misc.CopyArray(l);
            q = Misc.CopyArray(q_init);
            this.q_ranges = Misc.CopyArray(q_ranges);

            this.DH = DH;
            DH_Init();

            this.Goal = Goal;

            Joints = new List<Point[]> { DKP };
        }

        public Manipulator(Manipulator Source)
        {
            Base = Source.Base;
            l = Misc.CopyArray(Source.l);
            q = Misc.CopyArray(Source.q);
            q_ranges = Misc.CopyArray(Source.q_ranges);

            DH = Misc.CopyArray(Source.DH);
            DH_Init();

            Goal = Source.Goal;

            Joints = new List<Point[]>(Source.Joints);
        }

        public Manipulator(ManipData data)
        {
            Base = new Point(data.Base.X, data.Base.Y, data.Base.Z);

            l = new double[data.N];
            q = new double[data.N];
            q_ranges = new double[data.N, 2];
            DH = new TupleDH[data.N];
            for (int i = 0; i < data.N; i++)
            {
                l[i] = data.l[i];
                q[i] = data.q[i];

                q_ranges[i, 0] = data.q_ranges[i].X;
                q_ranges[i, 1] = data.q_ranges[i].Y;

                int index = i;
                DH[i] = new TupleDH((m) => { return m.q[index] + data.DH[index].X * Math.PI / 180; }, data.DH[i].Y, data.DH[i].Z * Math.PI / 180, data.DH[i].W);
            }
            DH_Init();

            Goal = new Point(data.Goal.X, data.Goal.Y, data.Goal.Z);

            Joints = new List<Point[]> { DKP };

            States = new Dictionary<string, bool>
            {
                { "Goal", false },
                { "Attractors", false },
                { "Path", false }
            };
        }

        public void DH_Init()
        {
            DH_Z();
            DH_X();
        }

        private void DH_Z()
        {
            RZ = new List<Matrix>();
            TZ = new List<Matrix>();
            foreach (var param in DH)
            {
                double theta = param.theta(this), d = param.d;

                Matrix rz = new Matrix(new double[3, 3]
                {
                    { Math.Cos(theta), 0, -Math.Sin(theta) },
                    { 0, 1, 0 },
                    { Math.Sin(theta), 0, Math.Cos(theta) },
                });
                RZ.Add(rz);

                Matrix tz = new Matrix(new double[3, 1]
                {
                    { 0 },
                    { d },
                    { 0 }
                });
                TZ.Add(tz);
            }
        }

        private void DH_X()
        {
            RX = new List<Matrix>();
            TX = new List<Matrix>();
            foreach (var param in DH)
            {
                double alpha = param.alpha, r = param.r;

                Matrix rx = new Matrix(new double[3, 3]
                {
                    { 1, 0, 0 },
                    { 0, Math.Cos(alpha), -Math.Sin(alpha) },
                    { 0, Math.Sin(alpha), Math.Cos(alpha) }
                });
                RX.Add(rx);

                Matrix tx = new Matrix(new double[3, 1]
                {
                    { r },
                    { 0 },
                    { 0 }
                });
                TX.Add(tx);
            }
        }

        public Point GripperPos
        {
            get
            {
                DH_Z();

                Matrix R = new Matrix(new double[3, 3]
                {
                    { 1, 0, 0 },
                    { 0, 1, 0 },
                    { 0, 0, 1 }
                });
                Matrix T = new Matrix(new double[3, 1]
                {
                    { Base.x },
                    { Base.y },
                    { Base.z }
                });
                for (int i = 0; i < DH.Length; i++)
                {
                    if (DH[i].d != 0)
                    {
                        T += R * TZ[i];
                    }
                    if (DH[i].theta(this) != 0)
                    {
                        R *= RZ[i];
                    }

                    if (DH[i].r != 0)
                    {
                        T += R * TX[i];
                    }
                    if (DH[i].alpha != 0)
                    {
                        R *= RX[i];
                    }
                }

                return new Point(T[0, 0], T[1, 0], T[2, 0]);
            }
        }

        public Point[] DKP
        {
            get
            {
                DH_Z();

                Point[] Joints = new Point[DH.Length + 1];
                Joints[0] = Base;
                
                Matrix R = new Matrix(new double[3, 3]
                {
                    { 1, 0, 0 },
                    { 0, 1, 0 },
                    { 0, 0, 1 }
                });
                Matrix T = new Matrix(new double[3, 1]
                {
                    { Base.x },
                    { Base.y },
                    { Base.z }
                });
                for (int i = 1; i < DH.Length + 1; i++)
                {
                    if (DH[i - 1].d != 0)
                    {
                        T += R * TZ[i - 1];
                    }
                    if (DH[i - 1].theta(this) != 0)
                    {
                        R *= RZ[i - 1];
                    }

                    if (DH[i - 1].r != 0)
                    {
                        T += R * TX[i - 1];
                    }
                    if (DH[i - 1].alpha != 0)
                    {
                        R *= RX[i - 1];
                    }

                    Joints[i] = new Point(T[0, 0], T[1, 0], T[2, 0]);
                }

                return Joints;
            }
        }

        public bool InWorkspace(Point point)
        {
            if (point.Distance - Base.Distance > l.Sum())
                return false;
            else
                return true;
        }

        public double DistanceTo(Point p)
        {
            return new Vector(GripperPos, p).Length;
        }
    }

    public interface ICollider
    {
        ColliderShape Shape { get; }
        Point[] Data { get; set; }
        Point Center { get; set; }

        void Draw();
    }

    public enum ColliderShape
    {
        Box,
        Sphere
    }

    class Box : ICollider
    {
        public ColliderShape Shape { get { return ColliderShape.Box; } }
        public Point[] Data { get; set; }
        public Point Center { get; set; }

        public Box(Point[] obst)
        {
            // retrieving boundary points
            double Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0, Zmin = 0, Zmax = 0;
            for (int i = 0; i < obst.Length; i++)
            {
                if (i == 0)
                {
                    Xmin = Xmax = obst[i].x;
                    Ymin = Ymax = obst[i].y;
                    Zmin = Zmax = obst[i].z;
                }
                else
                {
                    if (obst[i].x < Xmin)
                        Xmin = obst[i].x;
                    if (obst[i].x > Xmax)
                        Xmax = obst[i].x;
                    if (obst[i].y < Ymin)
                        Ymin = obst[i].y;
                    if (obst[i].y > Ymax)
                        Ymax = obst[i].y;
                    if (obst[i].z > Zmax)
                        Zmax = obst[i].z;
                    if (obst[i].z < Zmin)
                        Zmin = obst[i].z;
                }
            }

            // data
            Data = new Point[8]
            {
                        new Point(Xmin, Ymin, Zmin),
                        new Point(Xmax, Ymin, Zmin),
                        new Point(Xmax, Ymin, Zmax),
                        new Point(Xmin, Ymin, Zmax),
                        new Point(Xmin, Ymax, Zmin),
                        new Point(Xmax, Ymax, Zmin),
                        new Point(Xmax, Ymax, Zmax),
                        new Point(Xmin, Ymax, Zmax)
            };

            // central point
            Center = new Point((Xmax + Xmin) / 2, (Ymax + Ymin) / 2, (Zmax + Zmin) / 2);
        }

        public void Draw()
        {
            GL.DrawElements(BeginMode.LineStrip, 16, DrawElementsType.UnsignedInt, 0);
        }
    }

    class Sphere : ICollider
    {
        public ColliderShape Shape { get { return ColliderShape.Sphere; } }
        public Point[] Data { get; set; }
        public Point Center { get; set; }
        public double Radius;

        public uint[] indicesLongitude;

        public Sphere(Point[] obst)
        {
            // central point
            Center = Point.Zero;
            for (int i = 0; i < obst.Length; i++)
            {
                Center += obst[i];
            }
            Center /= obst.Length;

            // radius
            Radius = 0;
            for (int i = 0; i < obst.Length; i++)
            {
                double rNew = obst[i].DistanceTo(Center);
                if (rNew > Radius)
                    Radius = rNew;
            }

            // data
            int levels = 5, pointsNum = 20;
            Data = new Point[2 + levels * pointsNum];
            double y = Radius;
            Data[0] = new Point(0, y, 0) + Center;
            for (int i = 0; i < levels; i++)
            {
                y -= 2 * Radius / (levels + 1);
                double levelRadius = Math.Sqrt(Radius * Radius - y * y);
                for (int j = 0; j < pointsNum; j++)
                {
                    double angle = j * 2 * Math.PI / pointsNum;
                    double x = levelRadius * Math.Cos(angle),
                           z = levelRadius * Math.Sin(angle);
                    Data[1 + i * pointsNum + j] = new Point(x, y, z) + Center;
                }
            }
            y = -Radius;
            Data[Data.Length - 1] = new Point(0, y, 0) + Center;

            List<uint> indices = new List<uint>();
            for (uint j = 0; j < 20; j++)
            {
                indices.Add(0);
                for (uint k = 0; k < 5; k++)
                {
                    indices.Add(1 + j + k * 20);
                }
                for (uint k = 0; k < 5 + 1; k++)
                {
                    indices.Add(1 + 5 * 20 - j - k * 20);
                }
            }
            indicesLongitude = indices.ToArray();
        }

        public void Draw()
        {
            for (int i = 0; i < 5; i++)
            {
                GL.DrawArrays(PrimitiveType.LineLoop, 1 + i * 20, 20);
            }
        }
    }

    class Obstacle
    {
        public Point[] Data { get; set; }
        public ICollider Collider;

        public Obstacle(Point[] data, ColliderShape shape)
        {
            Data = new Point[data.Length];
            Array.Copy(data, Data, data.Length);
            
            switch (shape)
            {
                case ColliderShape.Box:
                    Collider = new Box(Data);
                    break;
                case ColliderShape.Sphere:
                    Collider = new Sphere(Data);
                    break;
            }
        }

        public bool Contains(Point p)
        {
            switch (Collider.Shape)
            {
                case ColliderShape.Box:
                    if (p.x >= Collider.Data[0].x && p.x <= Collider.Data[1].x &&
                        p.y >= Collider.Data[0].y && p.y <= Collider.Data[4].y &&
                        p.z >= Collider.Data[0].z && p.z <= Collider.Data[2].z)
                        return true;
                    else
                        return false;
                case ColliderShape.Sphere:
                    if (Collider.Center.DistanceTo(p) <= ((Sphere)Collider).Radius)
                        return true;
                    else
                        return false;
            }

            return false;
        }
    }
}
