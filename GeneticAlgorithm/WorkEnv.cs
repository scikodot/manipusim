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
        public double[] Links, q;
        public double[,] StepRanges;

        public Manipulator()
        {

        }

        public Manipulator(Point Base, double[] Links, double[] q, double[,] StepRanges)
        {
            this.Base = Base;
            this.Links = Misc.CopyArray(Links);
            this.q = Misc.CopyArray(q);
            this.StepRanges = Misc.CopyArray(StepRanges);
        }

        public Manipulator(Manipulator Source)
        {
            Base = Source.Base;
            Links = Misc.CopyArray(Source.Links);
            q = Misc.CopyArray(Source.q);
            StepRanges = Misc.CopyArray(Source.StepRanges);
        }

        public double[] qAbs
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
        }

        public Point[] Joints
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
                        Links[i - 1] * Math.Cos(q_abs[i - 1]),
                        Links[i - 1] * Math.Sin(q_abs[i - 1]),
                        0
                    );
                    if (i > 1)
                    {
                        Joints[i] += Joints[i - 1];
                    }
                }

                return Joints;
            }
        }

        public Point GripperPos
        {
            get
            {
                double[] q_abs = qAbs;
                return new Point
                (
                    q_abs.Zip(Links, (t, s) => { return s * Math.Cos(t); }).Sum(),
                    q_abs.Zip(Links, (t, s) => { return s * Math.Sin(t); }).Sum(),
                    0
                );
            }
        }

        public bool InWorkingRange(Point point)
        {
            if (point.Distance - Base.Distance > Links.Sum())
                return false;
            else
                return true;
        }

        public double DistanceTo(Point p)
        {
            return new Vector(p.x - GripperPos.x, p.y - GripperPos.y, 0).Length;
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
