using System;
using System.Collections.Generic;
using System.Linq;
using Graphics;
using Logic.PathPlanning;

namespace Logic
{
    public struct TupleDH
    {
        public Joint joint;
        public float theta
        {
            get { return joint.q + thetaOffset; }
        }

        public float thetaOffset;
        public float d;
        public float alpha;
        public float r;

        public TupleDH(float thetaOffset, float d, float alpha, float r)
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
        public int Vector3s_num;

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

        public float q;
        public float[] qRanges;

        public ImpDualQuat State;

        public Vector3 Position { get; set; }  // TODO: implement
        public Vector3 Axis { get; set; }

        public Joint() { }

        public Joint(JointData data)
        {
            Model = data.Model;
            Length = data.Length;
            q = data.q;
            qRanges = new float[2] { data.q_ranges.X, data.q_ranges.Y };
        }

        public Joint Copy(bool deep = false)
        {
            return (Joint)MemberwiseClone();
        }
    }

    public class Manipulator
    {
        public Vector3 Base;

        public Link[] Links;
        public Joint[] Joints;
        public TupleDH[] DH;
        public float WorkspaceRadius;

        public Vector3 Goal;
        public List<Vector3> Path;
        public List<Vector> Configs;
        public Tree Tree;
        public List<Tree.Node> Buffer = new List<Tree.Node>();
        public List<Attractor> GoodAttractors, BadAttractors;
        public Vector3[] points;
        public Dictionary<string, bool> States;

        private int posCounter = 0;

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

            Base = new Vector3(Joints[0].Model.Position.X, Joints[0].Model.Position.Y, Joints[0].Model.Position.Z);

            this.DH = DH;
            for (int i = 0; i < DH.Length; i++)
            {
                DH[i].joint = Joints[i];
            }
            UpdateState();

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
            UpdateState();

            WorkspaceRadius = source.WorkspaceRadius;

            Goal = source.Goal;  // TODO: review referencing

            States = new Dictionary<string, bool>(source.States);

            if (source.Path != null)
                Path = new List<Vector3>(source.Path);
        }

        public void UpdateState()
        {
            Joints[0].Axis = Vector3.UnitY;
            Joints[0].Position = Vector3.Zero;

            var quat = ImpDualQuat.Zero;

            for (int i = 0; i < Joints.Length; i++)
            {
                quat *= new ImpDualQuat(Vector3.UnitY, -DH[i].theta);

                Joints[i].State = quat;
                Joints[i].Model.Position = quat.Translation;

                quat *= new ImpDualQuat(DH[i].d * Vector3.UnitY);
                quat *= new ImpDualQuat(Vector3.UnitX, DH[i].alpha, DH[i].r * Vector3.UnitX);

                if (i < Joints.Length - 1)
                {
                    Joints[i + 1].Axis = quat.Rotate(Joints[0].Axis);
                    Joints[i + 1].Position = quat.Translation;
                }
                else
                {
                    GripperPos = quat.Translation;
                }
            }
        }

        public Vector3 GripperPos { get; set; }

        public Vector3[] DKP
        {
            get
            {
                List<Vector3> jointsPos = Joints.Select(x => x.Position).ToList();
                jointsPos.Add(GripperPos);  // TODO: gripper should be a part of Joints and treated similarly, even if it cannot rotate or anything
                return jointsPos.ToArray();
            }
        }

        public Vector q
        {
            get
            {
                return new Vector(Joints.Select(x => x.q).ToArray());
            }
            set
            {
                for (int i = 0; i < Joints.Length; i++)
                {
                    Joints[i].q = value[i];
                }

                UpdateState();
            }
        }

        public bool InWorkspace(Vector3 point)
        {
            if (point.DistanceTo(Vector3.Zero) - point.DistanceTo(Vector3.Zero) > WorkspaceRadius)
                return false;
            else
                return true;
        }

        public float DistanceTo(Vector3 p)
        {
            return new Vector3(GripperPos, p).Length;
        }

        public void Draw(Shader shader)
        {
            Dispatcher.UpdateConfig.Reset();

            if (Configs != null)
                q = Configs[posCounter < Configs.Count - 1 ? posCounter++ : posCounter];

            shader.Use();

            Matrix4 model;

            // joints
            for (int i = 0; i < Joints.Length; i++)
            {
                model = Joints[i].State.ToMatrix(true);

                shader.SetMatrix4("model", model);
                Joints[i].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);
            }

            var quat = ImpDualQuat.Zero;

            // links
            quat *= new ImpDualQuat(Joints[0].Length / 2 * Vector3.UnitY);

            // the order of multiplication is reversed, because the trans quat transforms the operand (quat) itself; it does not contribute in total transformation like other quats do
            var trans_quat = new ImpDualQuat(Joints[0].Axis, Joints[0].Position, -DH[0].theta);
            quat = trans_quat * quat;
            model = quat.ToMatrix(true);

            shader.SetMatrix4("model", model);
            Links[0].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);

            quat *= new ImpDualQuat(DH[0].d * Vector3.UnitY);

            trans_quat = new ImpDualQuat(Joints[1].Axis, Joints[1].Position, -(DH[1].theta + (float)Math.PI / 2));
            quat = trans_quat * quat;
            model = quat.ToMatrix(true);

            shader.SetMatrix4("model", model);
            Links[1].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);

            quat *= new ImpDualQuat(DH[1].r * Vector3.UnitY);

            trans_quat = new ImpDualQuat(Joints[2].Axis, Joints[2].Position, -DH[2].theta);
            quat = trans_quat * quat;
            model = quat.ToMatrix(true);

            shader.SetMatrix4("model", model);
            Links[2].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);

            Dispatcher.UpdateConfig.Set();
        }
    }
}
