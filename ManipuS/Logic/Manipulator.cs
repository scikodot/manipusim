using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Graphics;
using Logic.PathPlanning;
using OpenTK;

namespace Logic
{
    public struct TupleDH
    {
        public Joint joint;
        public double theta
        {
            get { return joint.q + thetaOffset; }
        }

        public double thetaOffset;
        public double d;
        public double alpha;
        public double r;

        public TupleDH(double thetaOffset, double d, double alpha, double r)
        {
            joint = null;  // TODO: probably better to use class?

            this.thetaOffset = thetaOffset;
            this.d = d;
            this.alpha = alpha;
            this.r = r;
        }

        public TupleDH Copy(bool deep = false)
        {
            return (TupleDH)MemberwiseClone();
        }
    }

    public struct LinkData
    {
        public Model Model;
        public float Length;
    }

    public struct JointData
    {
        public Model Model;
        public float Length;
        public float q;
        public System.Numerics.Vector2 q_ranges;
        public System.Numerics.Vector4 DH;

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

    public enum JointType
    {
        Prismatic,  // Translation
        Revolute,  // Rotation
        Cylindrical,  // Translation & rotation
        Spherical,  // Allows three degrees of rotational freedom about the center of the joint. Also known as a ball-and-socket joint
        Planar  // Allows relative translation on a plane and relative rotation about an axis perpendicular to the plane
    }

    public struct Link
    {
        public Model Model;
        public float Length;

        public Link(Model model, float length)
        {
            Model = model;
            Length = length;
        }

        public Link(LinkData data)
        {
            Model = data.Model;
            Length = data.Length;
        }
    }

    public class Joint
    {
        public Model Model;
        public float Length;

        public double q;
        public double[] qRanges;

        public Joint() { }

        public Joint(JointData data)
        {
            Model = data.Model;
            Length = data.Length;
            q = data.q;
            qRanges = new double[2] { data.q_ranges.X, data.q_ranges.Y };
        }

        public Joint Copy(bool deep = false)
        {
            return (Joint)MemberwiseClone();
        }
    }

    public class Manipulator
    {
        public Point Base;

        public Link[] Links;
        public Joint[] Joints;
        public TupleDH[] DH;
        public List<Matrix4> TransMatrices;
        public float WorkspaceRadius;

        public Point Goal;
        public List<Point> Path;
        public List<double[]> Configs;
        public Tree Tree;
        public List<Tree.Node> Buffer = new List<Tree.Node>();
        public List<Attractor> GoodAttractors, BadAttractors;
        public Point[] points;
        public Dictionary<string, bool> States;

        public Manipulator(LinkData[] links, JointData[] joints, TupleDH[] DH)
        {
            Links = new Link[links.Length];
            for (int i = 0; i < links.Length; i++)
            {
                Links[i] = new Link(links[i]);
            }

            Joints = new Joint[joints.Length];
            for (int i = 0; i < joints.Length; i++)
            {
                Joints[i] = new Joint(joints[i]);
            }

            Base = new Point(Joints[0].Model.Position.X, Joints[0].Model.Position.Y, Joints[0].Model.Position.Z);

            this.DH = DH;
            for (int i = 0; i < DH.Length; i++)
            {
                DH[i].joint = Joints[i];
            }
            UpdateTransMatrices();

            WorkspaceRadius = Links.Sum((link) => { return link.Length; }) + Joints.Sum((joint) => { return joint.Length; });

            States = new Dictionary<string, bool>
            {
                { "Goal", false },
                { "Attractors", false },
                { "Path", false }
            };
        }

        public Manipulator(Manipulator source)  // TODO: make a m.CreateCopy() method, not a constructor! that's much better
        {
            Links = Misc.CopyArray(source.Links);
            Joints = Array.ConvertAll(source.Joints, x => x.Copy());  //Misc.CopyArray(source.Joints);

            Base = source.Base;  // TODO: review referencing

            DH = Array.ConvertAll(source.DH, x => x.Copy());  //Misc.CopyArray(source.DH);
            for (int i = 0; i < DH.Length; i++)
            {
                DH[i].joint = Joints[i];
            }
            UpdateTransMatrices();

            WorkspaceRadius = source.WorkspaceRadius;

            Goal = source.Goal;  // TODO: review referencing

            States = new Dictionary<string, bool>(source.States);
        }


        public void UpdateTransMatrices()
        {
            TransMatrices = new List<Matrix4>();
            for (int i = 0; i < DH.Length; i++)
            {
                TransMatrices.Add(CreateTransMatrix(DH[i]));
            }
        }

        public static Matrix4 CreateTransMatrix(TupleDH DH)  // TODO: optimize; better to call with joint that contains its own DH table
        {
            float cosT = (float)Math.Cos(DH.theta);
            float sinT = (float)Math.Sin(DH.theta);
            float d = (float)DH.d;
            float cosA = (float)Math.Cos(DH.alpha);
            float sinA = (float)Math.Sin(DH.alpha);
            float r = (float)DH.r;

            return new Matrix4(
                new Vector4(cosT, -sinA * sinT, -sinT * cosA, r * cosT),
                new Vector4(0, cosA, -sinA, d),
                new Vector4(sinT, sinA * cosT, cosA * cosT, r * sinT),
                new Vector4(0, 0, 0, 1)
            );
        }

        public Point GripperPos
        {
            get
            {
                UpdateTransMatrices();

                Matrix4 pos = new Matrix4
                (
                    new Vector4(1, 0, 0, (float)Base.x),
                    new Vector4(0, 1, 0, (float)Base.y),
                    new Vector4(0, 0, 1, (float)Base.z),
                    new Vector4(0, 0, 0, 1)
                );

                for (int i = 0; i < DH.Length; i++)
                {
                    pos *= TransMatrices[i];
                }

                return new Point(pos[0, 3], pos[1, 3], pos[2, 3]);  // TODO: too big error (~2nd order)! optimize
            }
        }

        public Point[] DKP
        {
            get
            {
                UpdateTransMatrices();

                Point[] jointsPos = new Point[Joints.Length + 1];
                jointsPos[0] = Base;

                Matrix4 pos = new Matrix4
                (
                    new Vector4(1, 0, 0, (float)Base.x),
                    new Vector4(0, 1, 0, (float)Base.y),
                    new Vector4(0, 0, 1, (float)Base.z),
                    new Vector4(0, 0, 0, 1)
                );

                for (int i = 0; i < DH.Length; i++)
                {
                    pos *= TransMatrices[i];
                    jointsPos[i + 1] = new Point(pos[0, 3], pos[1, 3], pos[2, 3]);
                }

                return jointsPos;
            }
        }

        public double[] q
        {
            get
            {
                double[] q = new double[Joints.Length];
                for (int i = 0; i < Joints.Length; i++)
                {
                    q[i] = Joints[i].q;
                }
                return q;
            }
            set
            {
                for (int i = 0; i < Joints.Length; i++)
                {
                    Joints[i].q = value[i];
                }
            }
        }

        public bool InWorkspace(Point point)
        {
            if (point.Distance - Base.Distance > WorkspaceRadius)
                return false;
            else
                return true;
        }

        public double DistanceTo(Point p)
        {
            return new Vector(GripperPos, p).Length;
        }

        public void Draw(Shader shader)
        {
            Dispatcher.UpdateConfig.Reset();

            //time += (float)(Math.PI / 2 * e.Time);
            //joints[0].q = time;
            //joints[1].q = time;
            //joints[2].q = time;

            var model = Matrix4.Identity;

            Vector4[] axes = new Vector4[Links.Length];
            Vector4[] pos = new Vector4[Links.Length];

            axes[0] = Vector4.UnitY;
            pos[0] = new Vector4(Vector3.Zero, 1);

            shader.Use();

            // joints
            for (int i = 0; i < Links.Length; i++)  // TODO: all the manipulator structure logic should be incapsulated (elsewhere), so that it could be drawn with one-liner
            {
                model *= Matrix.RotateY((float)DH[i].theta);

                shader.SetMatrix4("model", model, true);
                Joints[i].Model.Position = new Vector3(model * new Vector4(Joints[0].Model.Position, 1.0f));
                Joints[i].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);

                model *= Matrix.Translate((float)DH[i].d * Vector3.UnitY);
                model *= Matrix.RotateX((float)DH[i].alpha);
                model *= Matrix.Translate((float)DH[i].r * Vector3.UnitX);

                if (i < Links.Length - 1)
                {
                    axes[i + 1] = model * axes[0];
                    pos[i + 1] = new Vector4(model.M14, model.M24, model.M34, 1);
                }
            }

            model = Matrix4.Identity;

            // TODO: replace matrices with quaternions!
            // links
            model *= Matrix.Translate(Joints[0].Length / 2 * Vector3.UnitY);
            var trans = Matrix.RotateCustomAnother(pos[0], axes[0], -(float)DH[0].theta);  // rotate link about custom axis; useful when dealing with complex (arbitrary) joints' actuation axes
            model = trans * model;  // the order of multiplication is reversed, because the trans matrix transforms the operand (model) itself; it does not contribute in total transformation like other matrices do
            model *= Matrix4.CreateScale(new Vector3(0.75f, 1, 0.75f));

            shader.SetMatrix4("model", model, true);
            Links[0].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);

            model *= Matrix.Translate((float)DH[0].d * Vector3.UnitY);
            trans = Matrix.RotateCustomAnother(pos[1], axes[1], -(float)(DH[1].theta + Math.PI / 2));
            model = trans * model;

            shader.SetMatrix4("model", model, true);
            Links[1].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);

            model *= Matrix.Translate((float)DH[1].r * Vector3.UnitY);
            trans = Matrix.RotateCustomAnother(pos[2], axes[2], -(float)DH[2].theta);
            model = trans * model;

            shader.SetMatrix4("model", model, true);
            Links[2].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);

            Dispatcher.UpdateConfig.Set();
        }
    }
}
