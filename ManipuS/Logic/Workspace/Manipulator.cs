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

        public TupleDH(System.Numerics.Vector4 data)
        {
            thetaOffset = data.X * (float)Math.PI / 180;
            d = data.Y;
            alpha = data.Z * (float)Math.PI / 180;
            r = data.W;
        }
    }

    public struct ManipData
    {
        public int N;
        public System.Numerics.Vector3 Base;
        public LinkData[] Links;
        public JointData[] Joints;
        public System.Numerics.Vector4[] DH;
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
        public System.Numerics.Vector2 q_ranges;
        public System.Numerics.Vector4 DH;
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
        public List<Attractor> Attractors;
        public Dictionary<string, bool> States;  // TODO: this is used weirdly; replace with something?

        public MotionController Controller { get; set; }

        public int posCounter = 0;

        public Manipulator(ManipData data)  //LinkData[] links, JointData[] joints, TupleDH[] DH)
        {
            Base = data.Base;
            Links = Array.ConvertAll(data.Links, x => new Link(x));
            Joints = Array.ConvertAll(data.Joints, x => new Joint(x));

            Joints[0].Axis = Vector3.UnitY;  // TODO: should be set from GUI, not hard-coded!
            Joints[0].Position = Base;

            var end = Joints.Length - 1;
            Joints[0].Axis = Joints[end].Axis = Vector3.UnitY;
            for (int i = 1; i < end; i++)
            {
                Joints[i].Axis = Vector3.UnitZ;
            }

            Joints[0].Position = Base;
            Joints[1].Position = Base + 1.4f * Vector3.UnitY;
            Joints[2].Position = Base + 2.8f * Vector3.UnitY;
            Joints[3].Position = Base + 4.1f * Vector3.UnitY;

            Joints[0].State = new ImpDualQuat(Joints[0].Position);

            for (int i = 0; i < data.DH.Length; i++)
            {
                Joints[i].DH = new TupleDH(data.DH[i]);
            }
            UpdateState(q);

            WorkspaceRadius = Links.Sum(link => link.Length) + Joints.Sum(joint => joint.Length);
            
            Goal = data.Goal;
            States = new Dictionary<string, bool>
            {
                { "Goal", false },
                { "Attractors", false },
                { "Path", false }
            };
        }

        public void UpdateState(Vector dq)
        {
            //var end = Joints.Length - 1;
            //var quat = new ImpDualQuat(Joints[0].Position);
            //for (int i = 0; i < end; i++)
            //{
            //    quat *= new ImpDualQuat(Vector3.UnitY, -(Joints[i].q + Joints[i].DH.thetaOffset));

            //    Joints[i].State = quat;
            //    Joints[i].Model.Position = quat.Translation;

            //    quat *= new ImpDualQuat(Joints[i].DH.d * Vector3.UnitY);
            //    quat *= new ImpDualQuat(Vector3.UnitX, Joints[i].DH.alpha, Joints[i].DH.r * Vector3.UnitX);

            //    Joints[i + 1].Axis = quat.Rotate(Joints[0].Axis);
            //    Joints[i + 1].Position = quat.Translation;
            //}

            //quat *= new ImpDualQuat(Vector3.UnitY, -(Joints[end].q + Joints[end].DH.thetaOffset));
            //quat *= new ImpDualQuat(Vector3.UnitX, Joints[end].DH.alpha);

            //Joints[end].State = quat;
            //Joints[end].Model.Position = quat.Translation;

            //Joints[end].Axis = quat.Rotate(Joints[0].Axis);
            //Joints[end].Position = quat.Translation;

            //var end = Joints.Length - 1;
            //ImpDualQuat quat = Joints[0].State;
            //ImpDualQuat quatAlign = ImpDualQuat.Zero;
            //for (int i = 0; i <= end; i++)
            //{
            //    quat *= new ImpDualQuat(Joints[i].Axis, -dq[i]);

            //    Joints[i].State = quat;

            //    Joints[i].Axis = Joints[i].State.Rotate(Joints[0].Axis);

            //    var angle = (float)Math.Acos(Vector3.Dot(Vector3.UnitZ, Joints[i].Axis));

            //    Joints[i].Position = Joints[i].Model.Position = Joints[i].State.Translation;

            //    if (i < end)
            //    {
            //        quat *= new ImpDualQuat(Joints[i + 1].Position - Joints[i].Position);
            //        //quatAlign = new ImpDualQuat(
            //        //    Vector3.Cross(Joints[i].Axis, Joints[i + 1].Axis).Normalized,
            //        //    (float)Math.Acos(Vector3.Dot(Joints[i].Axis, Joints[i + 1].Axis)));

            //        //Joints[i + 1].State *= quatAlign;
            //    }
            //}

            Vector3[] initAxes = new Vector3[]
            {
                Vector3.UnitY,
                Vector3.UnitZ,
                Vector3.UnitZ,
                Vector3.UnitY
            };

            Vector3[] initPos = new Vector3[]
            {
                Base,
                Base + 1.4f * Vector3.UnitY,
                Base + 2.8f * Vector3.UnitY,
                Base + 4.1f * Vector3.UnitY
            };

            ImpDualQuat init = new ImpDualQuat(Base);

            ImpDualQuat quat = init;
            for (int i = 0; i < Joints.Length; i++)
            {
                quat = init;

                for (int j = 0; j < i; j++)
                {
                    quat *= new ImpDualQuat(initPos[j + 1] - initPos[j]);
                    quat *= new ImpDualQuat(initAxes[j], quat.Translation, initPos[j], -Joints[j].q);
                    //quat *= new ImpDualQuat(Joints[j].Axis, -Joints[j].q);
                }

                quat *= new ImpDualQuat(initAxes[i], -Joints[i].q);

                Vector3 axis = initAxes[i];

                var axisAlign = Vector3.Cross(Joints[0].Axis, axis).Normalized;
                var angleAlign = (float)Math.Acos(Vector3.Dot(Joints[0].Axis, axis));
                quat *= new ImpDualQuat(axisAlign, angleAlign);

                Joints[i].State = quat;
                Joints[i].Axis = quat.Rotate(initAxes[0]);
                Joints[i].Position = quat.Translation;
            }

            //for (int i = 0; i < Joints.Length; i++)
            //{
            //    quat = Joints[i].State * new ImpDualQuat(initAxes[i], -Joints[i].q);

            //    //Vector3 axis = initAxes[i];

            //    //var axisAlign = Vector3.Cross(Joints[0].Axis, axis).Normalized;
            //    //var angleAlign = (float)Math.Acos(Vector3.Dot(Joints[0].Axis, axis));
            //    //quat *= new ImpDualQuat(axisAlign, angleAlign);

            //    Joints[i].State = quat;
            //    Joints[i].Axis = quat.Rotate(initAxes[0]);
            //    Joints[i].Position = quat.Translation;

            //    for (int j = i + 1; j < Joints.Length; j++)
            //    {
            //        quat = Joints[i].State * new ImpDualQuat(initPos[j] - initPos[i]);
            //        quat *= new ImpDualQuat(initAxes[i], quat.Translation, initPos[i], -Joints[i].q);

            //        var axis = initAxes[j];

            //        var axisAlign = Vector3.Cross(Joints[0].Axis, axis).Normalized;
            //        var angleAlign = (float)Math.Acos(Vector3.Dot(Joints[0].Axis, axis));
            //        quat *= new ImpDualQuat(axisAlign, angleAlign);

            //        Joints[j].State = quat;
            //        Joints[j].Axis = quat.Rotate(initAxes[0]);
            //        Joints[j].Position = quat.Translation;
            //    }
            //}

            //quat *= new ImpDualQuat(Joints[0].Axis, -Joints[0].q, Joints[1].Position - Joints[0].Position);

            //Vector3 axis = initAxes[1];

            //var axisTemp = Vector3.Cross(Joints[0].Axis, axis).Normalized;
            //var angle = (float)Math.Acos(Vector3.Dot(Joints[0].Axis, axis));
            //quat *= new ImpDualQuat(axisTemp, angle);

            //Joints[1].State = quat;
            //Joints[1].Axis = quat.Rotate(initAxes[0]);
            //Joints[1].Position = quat.Translation;

            //quat *= new ImpDualQuat(initPos[1] - initPos[0]);
            //quat *= new ImpDualQuat(initAxes[0], quat.Translation, initPos[0], -Joints[0].q);
            //quat *= new ImpDualQuat(initPos[2] - initPos[1]);
            //quat *= new ImpDualQuat(initAxes[1], quat.Translation, initPos[1], -Joints[1].q);

            //Vector3 axis = initAxes[2];

            //var axisTemp = Vector3.Cross(Joints[0].Axis, axis).Normalized;
            //var angle = (float)Math.Acos(Vector3.Dot(Joints[0].Axis, axis));
            //quat *= new ImpDualQuat(axisTemp, angle);

            //Joints[1].State = quat;
            //Joints[1].Axis = quat.Rotate(initAxes[0]);
            //Joints[1].Position = quat.Translation;
        }

        public Vector3 GripperPos => Joints[Joints.Length - 1].Position;

        public Vector3[] DKP => Joints.Select(x => x.Position).ToArray();

        public Vector q
        {
            get => new Vector(Array.ConvertAll(Joints, x => x.q));
            set
            {
                var dq = new Vector(Joints.Length);
                for (int i = 0; i < Joints.Length; i++)
                {
                    dq[i] = value[i] - Joints[i].q;
                    Joints[i].q = value[i];
                }

                UpdateState(dq);
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

            q += new Vector(0.016f, 0.016f, 0.016f, 0.016f);

            // joints
            for (int i = 0; i < Joints.Length; i++)
            {
                model = Joints[i].State.ToMatrix(true);
                shader.SetMatrix4("model", model);
                Joints[i].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);
            }

            var quat = new ImpDualQuat(Joints[0].Position);

            // links
            quat *= new ImpDualQuat(Joints[0].Length / 2 * Vector3.UnitY);  // TODO: move links logic to UpdateState(); here they should only be displayed with appropriate matrices

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

            for (int i = 2; i < Links.Length; i++)
            {
                quat *= new ImpDualQuat(Joints[i - 1].DH.r * Vector3.UnitY);
                trans_quat = new ImpDualQuat(Joints[i].Axis, Joints[i].Position, -(Joints[i].q + Joints[i].DH.thetaOffset));
                quat = trans_quat * quat;

                model = quat.ToMatrix(true);
                shader.SetMatrix4("model", model);
                Links[i].Model.Draw(shader, MeshMode.Solid | MeshMode.Wireframe);
            }
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
