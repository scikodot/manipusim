using System;
using System.Collections.Generic;
using System.Linq;
using Graphics;
using Logic.PathPlanning;

namespace Logic
{
    public struct ManipData
    {
        public int N;
        public LinkData[] Links;
        public JointData[] Joints;
        public System.Numerics.Vector3[] JointAxes;
        public System.Numerics.Vector3[] JointPositions;
        public System.Numerics.Vector3 Goal;
        public bool ShowTree;
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
        public System.Numerics.Vector2 qRanges;
    }

    public struct ObstData
    {
        public float Radius;
        public System.Numerics.Vector3 Center;
        public int PointsNum;
        public bool ShowCollider;
    }

    public struct AlgData
    {
        public int InverseKinematicsSolverID;
        public float StepSize;
        public float Precision;
        public int MaxTime;

        public int PathPlannerID;
        public int AttrNum;
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

    public class Link  // TODO: probably class would be better? it's an object after all, not a set of data
    {
        public Model Model;
        public float Length;

        public ImpDualQuat State;

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

        public Link ShallowCopy()
        {
            return (Link)MemberwiseClone();
        }
    }

    public class Joint
    {
        public Model Model;
        public float Length;

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
            qRanges = new float[2] { data.qRanges.X, data.qRanges.Y };
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

        public Vector3[] InitialAxes;
        public Vector3[] InitialPositions;

        public Vector3 Goal;
        public List<Vector3> Path;
        public List<Vector> Configs;
        public Tree Tree;
        public List<Attractor> Attractors;
        public Dictionary<string, bool> States;  // TODO: this is used weirdly; replace with something?

        public MotionController Controller { get; set; }

        public int posCounter = 0;

        public Manipulator(ManipData data)
        {
            Base = data.JointPositions[0];
            Links = Array.ConvertAll(data.Links, x => new Link(x));
            Joints = Array.ConvertAll(data.Joints, x => new Joint(x));

            InitialAxes = Array.ConvertAll(data.JointAxes, x => (Vector3)x);  //Array.ConvertAll(data.Joints, x => (Vector3)x.Axis);
            InitialPositions = Array.ConvertAll(data.JointPositions, x => (Vector3)x); //Array.ConvertAll(data.Joints, x => (Vector3)x.Position);

            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i].Axis = data.JointAxes[i];
                Joints[i].Position = data.JointPositions[i];
            }

            UpdateState();

            WorkspaceRadius = Links.Sum(link => link.Length) + Joints.Sum(joint => joint.Length);
            
            Goal = data.Goal;
            States = new Dictionary<string, bool>
            {
                { "Goal", false },
                { "Attractors", false },
                { "Path", false }
            };
        }

        public void UpdateState()
        {
            ImpDualQuat quat;
            ImpDualQuat init = new ImpDualQuat(Base);

            // joints
            for (int i = 0; i < Joints.Length; i++)
            {
                quat = init;

                for (int j = 0; j < i; j++)
                {
                    quat *= new ImpDualQuat(InitialPositions[j + 1] - InitialPositions[j]);  // TODO: this *may* cause inappropriate behaviour; consider initPos[j + 1] - initPos[i]
                    quat *= new ImpDualQuat(quat.Conjugate, InitialAxes[j], quat.Translation, Joints[j].Position, -Joints[j].q);  // TODO: optimize; probably, conjugation can be avoided
                }

                quat *= new ImpDualQuat(InitialAxes[i], -Joints[i].q);

                quat *= ImpDualQuat.Align(Joints[0].Axis, InitialAxes[i]);

                Joints[i].State = quat;
                Joints[i].Axis = quat.Rotate(Joints[0].Axis);
                Joints[i].Position = quat.Translation;
            }

            // links
            for (int i = 0; i < Links.Length; i++)
            {
                quat = init;

                for (int j = 0; j <= i; j++)
                {
                    quat *= j == 0 ?
                        new ImpDualQuat(Joints[0].Length / 2 * Vector3.UnitY) :
                        new ImpDualQuat(InitialPositions[j] - InitialPositions[j - 1]);
                    quat *= new ImpDualQuat(quat.Conjugate, InitialAxes[j], quat.Translation, Joints[j].Position, -Joints[j].q);
                }

                Links[i].State = quat;
            }
        }

        public Vector3 GripperPos => Joints[Joints.Length - 1].Position;

        public Vector3[] DKP => Joints.Select(x => x.Position).ToArray();

        public Vector q
        {
            get => new Vector(Array.ConvertAll(Joints, x => x.q));
            set
            {
                for (int i = 0; i < Joints.Length; i++)
                {
                    Joints[i].q = value[i];
                }

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

            //q += new Vector(0.016f, 0.016f, 0.016f, 0.016f, 0.016f, 0.016f, 0.016f);

            // joints
            for (int i = 0; i < Joints.Length; i++)
            {
                model = Joints[i].State.ToMatrix(true);
                shader.SetMatrix4("model", model);
                Joints[i].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);
            }

            // links
            for (int i = 0; i < Links.Length; i++)
            {
                model = Links[i].State.ToMatrix(true);
                shader.SetMatrix4("model", model);
                Links[i].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);
            }
        }

        public Manipulator DeepCopy()
        {
            Manipulator manip = (Manipulator)MemberwiseClone();

            manip.Links = Array.ConvertAll(Links, x => x.ShallowCopy());
            manip.Joints = Array.ConvertAll(Joints, x => x.ShallowCopy());

            manip.States = new Dictionary<string, bool>(States);

            if (Path != null)
                manip.Path = new List<Vector3>(Path);

            return manip;
        }
    }
}
