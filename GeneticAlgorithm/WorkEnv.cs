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
            this.Links = Links;
            this.q = q;
            this.StepRanges = StepRanges;
        }

        public Manipulator(Manipulator Source)
        {
            Base = Source.Base;
            Links = Source.Links;
            q = Source.q;
            StepRanges = Source.StepRanges;
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
                        Links[i - 1] * Math.Sin(q_abs[i - 1])
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
                    q_abs.Zip(Links, (t, s) => { return s * Math.Sin(t); }).Sum()
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
            return new Vector(p.x - GripperPos.x, p.y - GripperPos.y).Length;
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
                    Bounding = new Point[4];
                    Point Xmin, Xmax, Ymin, Ymax;
                    Xmin = Xmax = Ymin = Ymax = Point.Zero;
                    for (int i = 0; i < Data.Length; i++)
                    {
                        if (i == 0)
                        {
                            Xmin = Xmax = Ymin = Ymax = Data[i];
                        }
                        else
                        {
                            if (Data[i].x < Xmin.x)
                                Xmin = Data[i];
                            if (Data[i].x > Xmax.x)
                                Xmax = Data[i];
                            if (Data[i].y < Ymin.y)
                                Ymin = Data[i];
                            if (Data[i].y > Ymax.y)
                                Ymax = Data[i];
                        }
                    }

                    Bounding[0] = new Point(Xmin.x, Ymin.y);
                    Bounding[1] = new Point(Xmin.x, Ymax.y);
                    Bounding[2] = new Point(Xmax.x, Ymax.y);
                    Bounding[3] = new Point(Xmax.x, Ymin.y);

                    Center = new Point((Xmax.x + Xmin.x) / 2, (Ymax.y + Ymin.y) / 2);
                    break;
            }
        }

        public bool Contains(Point p)
        {
            if (p.x >= Bounding[0].x && p.x <= Bounding[2].x && p.y >= Bounding[0].y && p.y <= Bounding[1].y)
                return true;
            else
                return false;
        }
    }
}
