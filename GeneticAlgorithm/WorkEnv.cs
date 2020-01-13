using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Helper;

namespace WorkEnv
{    
    public class Manipulator
    {
        public Point Base;
        public double[] l, q;
        public double[,] q_ranges;
        public Tuple<Func<Manipulator, double>, double, double, double>[] DH;

        public List<Matrix> MatZ, MatX;

        public Manipulator()
        {

        }

        public Manipulator(Point Base, double[] l, double[] q_init, double[,] q_ranges, Tuple<Func<Manipulator, double>, double, double, double>[] DH)
        {
            this.Base = Base;
            this.l = Misc.CopyArray(l);
            q = Misc.CopyArray(q_init);
            this.q_ranges = Misc.CopyArray(q_ranges);

            this.DH = DH;
            DH_Init();

            /*MatZ = new List<Matrix>();
            MatX = new List<Matrix>();
            foreach (var param in DH)
            {
                double theta = param[0], d = param[1], alpha = param[2], r = param[3];

                Matrix Z = new Matrix(new double[4, 4]
                {
                    { Math.Cos(theta), -Math.Sin(theta), 0, 0 },
                    { Math.Sin(theta), Math.Cos(theta), 0, 0 },
                    { 0, 0, 1, d },
                    { 0, 0, 0, 1 }
                });
                MatZ.Add(Z);

                Matrix X = new Matrix(new double[4, 4]
                {
                    { 1, 0, 0, r },
                    { 0, Math.Cos(alpha), -Math.Sin(alpha), 0 },
                    { 0, Math.Sin(alpha), Math.Cos(alpha), 0 },
                    { 0, 0, 0, 1 }
                });
                MatX.Add(X);
            }*/
        }

        public Manipulator(Manipulator Source)
        {
            Base = Source.Base;
            l = Misc.CopyArray(Source.l);
            q = Misc.CopyArray(Source.q);
            q_ranges = Misc.CopyArray(Source.q_ranges);

            DH = Misc.CopyArray(Source.DH);
            DH_Init();
        }

        public void DH_Init()
        {
            MatZ = DH_Z();
            MatX = DH_X();
        }

        private List<Matrix> DH_Z()
        {
            List<Matrix> matZ = new List<Matrix>();
            foreach (var param in DH)
            {
                double theta = param.Item1(this), d = param.Item2;

                Matrix Z = new Matrix(new double[4, 4]
                {
                            { Math.Cos(theta), 0, -Math.Sin(theta), 0 },
                            { 0, 1, 0, d },
                            { Math.Sin(theta), 0, Math.Cos(theta), 0 },
                            { 0, 0, 0, 1 }
                });
                matZ.Add(Z);
            }
            /*for (int i = 0; i < DH.Length; i++)
            {
                double theta = (i == DH.Length - 1 && LockedGripper) ? 0 : q[i], d = DH[i].Item2;

                Matrix Z = new Matrix(new double[4, 4]
                {
                        { Math.Cos(theta), 0, -Math.Sin(theta), 0 },
                        { 0, 1, 0, d },
                        { Math.Sin(theta), 0, Math.Cos(theta), 0 },
                        { 0, 0, 0, 1 }
                });
                matZ.Add(Z);
            }*/

            return matZ;
        }

        private List<Matrix> DH_X()
        {
            List<Matrix> matX = new List<Matrix>();
            foreach (var param in DH)
            {
                double alpha = param.Item3, r = param.Item4;

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
        }

        /*public void UpdateZ()
        {
            if (MatZ.Count != 0)
            {
                for (int i = 0; i < MatZ.Count; i++)
                {
                    double S = (i == DH.Length - 1 && LockedGripper) ? 0 : Math.Sin(q[i]),
                           C = (i == DH.Length - 1 && LockedGripper) ? 1 : Math.Cos(q[i]);
                    MatZ[i][0, 0] = MatZ[i][2, 2] = C;
                    MatZ[i][0, 2] = -S;
                    MatZ[i][2, 0] = S;
                }
            }
        }*/

        /*public double[] qAbs
        {
            get
            {
                double[] res = new double[q.Length];
                for (int i = 0; i < q.Length; i++)
                {
                    res[i] = q[i] + (i > 0 ? res[i - 1] : 0);
                }
                return res;
            }
        }*/

        public Point[] DKP
        {
            get
            {
                MatZ = DH_Z();

                Point[] Joints = new Point[DH.Length + 1];
                Joints[0] = Base;

                Matrix Conv = new Matrix(new double[4, 4]
                {
                        { 1, 0, 0, Base.x },
                        { 0, 1, 0, Base.y },
                        { 0, 0, 1, Base.z },
                        { 0, 0, 0, 1 },
                });
                for (int i = 1; i < DH.Length + 1; i++)
                {
                    Conv *= MatZ[i - 1] * MatX[i - 1];
                    Joints[i] = new Point(Conv[0, 3], Conv[1, 3], Conv[2, 3]);
                }

                return Joints;
            }
        }

        /*public Point[] Joints
        {
            get
            {
                Point[] Joints = new Point[q.Length + 1];
                Joints[0] = Base;

                double[] q_abs = qAbs;
                for (int i = 1; i < q.Length + 1; i++)
                {
                    Joints[i] = new Point
                    (
                        l[i - 1] * Math.Cos(q_abs[i - 1]),
                        l[i - 1] * Math.Sin(q_abs[i - 1]),
                        0
                    );
                    if (i > 1)
                    {
                        Joints[i] += Joints[i - 1];
                    }
                }

                return Joints;
            }
        }*/

        public Point GripperPos
        {
            get
            {
                MatZ = DH_Z();

                Matrix Conv = new Matrix(new double[4, 4]
                {
                        { 1, 0, 0, Base.x },
                        { 0, 1, 0, Base.y },
                        { 0, 0, 1, Base.z },
                        { 0, 0, 0, 1 },
                });
                for (int i = 0; i < DH.Length; i++)
                {
                    Conv *= MatZ[i] * MatX[i];
                }

                return new Point(Conv[0, 3], Conv[1, 3], Conv[2, 3]);
                /*double[] q_abs = qAbs;
                return new Point
                (
                    q_abs.Zip(l, (t, s) => { return s * Math.Cos(t); }).Sum(),
                    q_abs.Zip(l, (t, s) => { return s * Math.Sin(t); }).Sum(),
                    0
                );*/
            }
        }

        public bool InWorkingRange(Point point)
        {
            if (point.Distance - Base.Distance > l.Sum())
                return false;
            else
                return true;
        }

        public double DistanceTo(Point p)
        {
            Vector vec = new Vector(GripperPos, p);
            return vec.Length;
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
