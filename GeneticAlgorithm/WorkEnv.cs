using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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

        public void DH_Init()
        {
            DH_Z();
            DH_X();
            //MatZ = DH_Z();
            //MatX = DH_X();
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

        /*private List<Matrix> DH_Z()
        {
            List<Matrix> matZ = new List<Matrix>();
            foreach (var param in DH)
            {
                double theta = param.theta(this), d = param.d;

                Matrix Z = new Matrix(new double[4, 4]
                {
                        { Math.Cos(theta), 0, -Math.Sin(theta), 0 },
                        { 0, 1, 0, d },
                        { Math.Sin(theta), 0, Math.Cos(theta), 0 },
                        { 0, 0, 0, 1 }
                });
                matZ.Add(Z);
            }

            return matZ;
        }*/

        /*private List<Matrix> DH_X()
        {
            List<Matrix> matX = new List<Matrix>();
            foreach (var param in DH)
            {
                double alpha = param.alpha, r = param.r;

                Matrix X = new Matrix(new double[4, 4]
                {
                        { 1, 0, 0, r },
                        { 0, Math.Cos(alpha), -Math.Sin(alpha), 0 },
                        { 0, Math.Sin(alpha), Math.Cos(alpha), 0 },
                        { 0, 0, 0, 1 }
                });
                matX.Add(X);
            }

            return matX;
        }*/

        public Point GripperPos
        {
            get
            {
                DH_Z();
                //MatZ = DH_Z();

                /*Matrix Conv = new Matrix(new double[4, 4]
                {
                        { 1, 0, 0, Base.x },
                        { 0, 1, 0, Base.y },
                        { 0, 0, 1, Base.z },
                        { 0, 0, 0, 1 },
                });*/
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
                    //Conv *= MatZ[i] * MatX[i];
                }

                return new Point(T[0, 0], T[1, 0], T[2, 0]);
                //return new Point(Conv[0, 3], Conv[1, 3], Conv[2, 3]);
            }
        }

        public Point[] DKP
        {
            get
            {
                DH_Z();
                //MatZ = DH_Z();

                Point[] Joints = new Point[DH.Length + 1];
                Joints[0] = Base;

                /*Matrix Conv = new Matrix(new double[4, 4]
                {
                        { 1, 0, 0, Base.x },
                        { 0, 1, 0, Base.y },
                        { 0, 0, 1, Base.z },
                        { 0, 0, 0, 1 },
                });*/
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
                    //Conv *= MatZ[i - 1] * MatX[i - 1];
                    //Joints[i] = new Point(Conv[0, 3], Conv[1, 3], Conv[2, 3]);
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

    class Obstacle
    {
        public Point[] Data, Bounding;
        public Point Center;

        public Obstacle(Point[] data, int shape = 0)
        {
            Data = new Point[data.Length];
            Array.Copy(data, Data, data.Length);
            CreateBounding(shape);
        }

        public void CreateBounding(int shape)
        {
            switch (shape)
            {
                //bounding box, i.e. square
                case 0:
                    Bounding = new Point[8];
                    double Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0, Zmin = 0, Zmax = 0;
                    for (int i = 0; i < Data.Length; i++)
                    {
                        if (i == 0)
                        {
                            Xmin = Xmax = Data[i].x;
                            Ymin = Ymax = Data[i].y;
                            Zmin = Zmax = Data[i].z;
                        }
                        else
                        {
                            if (Data[i].x < Xmin)
                                Xmin = Data[i].x;
                            if (Data[i].x > Xmax)
                                Xmax = Data[i].x;
                            if (Data[i].y < Ymin)
                                Ymin = Data[i].y;
                            if (Data[i].y > Ymax)
                                Ymax = Data[i].y;
                            if (Data[i].z > Zmax)
                                Zmax = Data[i].z;
                            if (Data[i].z < Zmin)
                                Zmin = Data[i].z;
                        }
                    }

                    Bounding[0] = new Point(Xmin, Ymin, Zmin);
                    Bounding[1] = new Point(Xmax, Ymin, Zmin);
                    Bounding[2] = new Point(Xmax, Ymin, Zmax);
                    Bounding[3] = new Point(Xmin, Ymin, Zmax);
                    Bounding[4] = new Point(Xmin, Ymax, Zmin);
                    Bounding[5] = new Point(Xmax, Ymax, Zmin);
                    Bounding[6] = new Point(Xmax, Ymax, Zmax);
                    Bounding[7] = new Point(Xmin, Ymax, Zmax);

                    Center = new Point((Xmax + Xmin) / 2, (Ymax + Ymin) / 2, (Zmax + Zmin) / 2);
                    break;
            }
        }

        public bool Contains(Point p)
        {
            if (p.x >= Bounding[0].x && p.x <= Bounding[1].x && 
                p.y >= Bounding[0].y && p.y <= Bounding[4].y && 
                p.z >= Bounding[0].z && p.z <= Bounding[2].z)
                return true;
            else
                return false;
        }
    }
}
