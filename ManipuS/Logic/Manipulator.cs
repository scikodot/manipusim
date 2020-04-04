using System;
using System.Collections.Generic;
using System.Linq;
using Graphics;
using Logic.PathPlanning;

namespace Logic
{
    public struct TupleDH
    {
        public float thetaOffset;
        public float d;
        public float alpha;
        public float r;

        public TupleDH(float thetaOffset, float d, float alpha, float r)
        {
            this.thetaOffset = thetaOffset;
            this.d = d;
            this.alpha = alpha;
            this.r = r;
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

    public struct Link  // TODO: probably class would be better? it's an object after all, not a set of data
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

        public TupleDH DH;

        public float q;
        public float[] qRanges;

        public ImpDualQuat State;

        public Vector3 Position { get; set; }
        public Vector3 Axis { get; set; }

        public Joint(JointData data)
        {
            Model = data.Model;
            Length = data.Length;
            q = data.q;
            qRanges = new float[2] { data.q_ranges.X, data.q_ranges.Y };
        }

        public Joint ShallowCopy()
        {
            return (Joint)MemberwiseClone();
        }
    }

    public class Manipulator
    {
        public Vector3 Base;
        public Link[] Links;
        public Joint[] Joints;
        public float WorkspaceRadius;

        public Vector3 Goal;
        public List<Vector3> Path;
        public List<Vector> Configs;
        public Tree Tree;
        public List<Attractor> GoodAttractors, BadAttractors;
        public Dictionary<string, bool> States;  // TODO: this is used weirdly; replace with something?

        public MotionController controller;

        public int posCounter = 0;

        public Manipulator(LinkData[] links, JointData[] joints, TupleDH[] DH)
        {
            Links = Array.ConvertAll(links, x => new Link(x));
            Joints = Array.ConvertAll(joints, x => new Joint(x));

            for (int i = 0; i < DH.Length; i++)
            {
                Joints[i].DH = DH[i];
            }
            UpdateState();

            Base = Joints[0].Position;

            WorkspaceRadius = Links.Sum(link => link.Length) + Joints.Sum(joint => joint.Length);

            States = new Dictionary<string, bool>
            {
                { "Goal", false },
                { "Attractors", false },
                { "Path", false }
            };
        }

        public void UpdateState()
        {
            Joints[0].Axis = Vector3.UnitY;
            Joints[0].Position = Vector3.Zero;

            var quat = ImpDualQuat.Zero;

            for (int i = 0; i < Joints.Length; i++)
            {
                quat *= new ImpDualQuat(Vector3.UnitY, -(Joints[i].q + Joints[i].DH.thetaOffset));

                Joints[i].State = quat;
                Joints[i].Model.Position = quat.Translation;

                quat *= new ImpDualQuat(Joints[i].DH.d * Vector3.UnitY);
                quat *= new ImpDualQuat(Vector3.UnitX, Joints[i].DH.alpha, Joints[i].DH.r * Vector3.UnitX);

                if (i < Joints.Length - 1)
                {
                    Joints[i + 1].Axis = quat.Rotate(Joints[0].Axis);
                    Joints[i + 1].Position = quat.Translation;
                }
                else
                {
                    GripperPos = quat.Translation;  // TODO: gripper should be a part of Joints and treated similarly
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
                return new Vector(Array.ConvertAll(Joints, x => x.q));
            }
            set
            {
                for (int i = 0; i < Joints.Length; i++)
                    Joints[i].q = value[i];

                UpdateState();
            }
        }

        public bool InWorkspace(Vector3 point)  // TODO: not used anywhere; fix
        {
            return point.DistanceTo(Vector3.Zero) - point.DistanceTo(Vector3.Zero) <= WorkspaceRadius;
        }

        public float DistanceTo(Vector3 p)
        {
            return new Vector3(GripperPos, p).Length;
        }

        public void Draw(Shader shader)
        {
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
            var trans_quat = new ImpDualQuat(Joints[0].Axis, Joints[0].Position, -(Joints[0].q + Joints[0].DH.thetaOffset));
            quat = trans_quat * quat;
            model = quat.ToMatrix(true);

            shader.SetMatrix4("model", model);
            Links[0].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);

            quat *= new ImpDualQuat(Joints[0].DH.d * Vector3.UnitY);

            trans_quat = new ImpDualQuat(Joints[1].Axis, Joints[1].Position, -(Joints[1].q + Joints[1].DH.thetaOffset + (float)Math.PI / 2));
            quat = trans_quat * quat;
            model = quat.ToMatrix(true);

            shader.SetMatrix4("model", model);
            Links[1].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);

            quat *= new ImpDualQuat(Joints[1].DH.r * Vector3.UnitY);

            trans_quat = new ImpDualQuat(Joints[2].Axis, Joints[2].Position, -(Joints[2].q + Joints[2].DH.thetaOffset));
            quat = trans_quat * quat;
            model = quat.ToMatrix(true);

            shader.SetMatrix4("model", model);
            Links[2].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);
        }

        public Manipulator DeepCopy()
        {
            Manipulator manip = (Manipulator)MemberwiseClone();

            Array.Copy(Links, manip.Links, Links.Length);
            manip.Joints = Array.ConvertAll(Joints, x => x.ShallowCopy());

            manip.States = new Dictionary<string, bool>(States);

            if (Path != null)
                manip.Path = new List<Vector3>(Path);

            return manip;
        }
    }
}
