using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

using Graphics;
using Logic.PathPlanning;

namespace Logic
{
    public struct ManipData
    {
        public int N;
        public LinkData[] Links;
        public JointData[] Joints;
        public Vector3[] JointAxes;
        public Vector3[] JointPositions;
        public Vector3 Goal;
        public bool ShowTree;
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

        public MotionController Controller { get; set; }

        public int posCounter = 0;

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        public Manipulator(ManipData data)
        {
            Base = data.JointPositions[0];
            Links = Array.ConvertAll(data.Links, x => new Link(x));
            Joints = Array.ConvertAll(data.Joints, x => new Joint(x));

            InitialAxes = data.JointAxes;
            InitialPositions = data.JointPositions;

            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i].Axis = data.JointAxes[i];
                Joints[i].Position = data.JointPositions[i];
            }

            _q = new Vector(Joints.Select(x => x.q).ToArray());

            UpdateState();

            WorkspaceRadius = Links.Sum(link => link.Length) + Joints.Sum(joint => joint.Length);
            
            Goal = data.Goal;
        }

        public void UpdateState()
        {
            DKP = new Vector3[Joints.Length];

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

                Joints[i].UpdateState(ref quat);

                Joints[i].Axis = quat.Rotate(Joints[0].Axis);
                Joints[i].Position = DKP[i] = quat.Translation;
            }

            //// links
            //for (int i = 0; i < Links.Length; i++)
            //{
            //    quat = init;

            //    for (int j = 0; j <= i; j++)
            //    {
            //        quat *= j == 0 ?
            //            new ImpDualQuat(Joints[0].Length / 2 * Vector3.UnitY) :
            //            new ImpDualQuat(InitialPositions[j] - InitialPositions[j - 1]);
            //        quat *= new ImpDualQuat(quat.Conjugate, InitialAxes[j], quat.Translation, Joints[j].Position, -Joints[j].q);
            //    }

            //    Links[i].UpdateState(ref quat);
            //}

            // gripper
            GripperPos = DKP[Joints.Length - 1];
        }

        public Vector3 GripperPos { get; private set; }

        public Vector3[] DKP { get; private set; }

        private Vector _q;
        public Vector q
        {
            get => _q;
            set
            {
                _q = value;
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
            return GripperPos.DistanceTo(p);
        }

        public void Render(Shader shader)
        {
            if (Configs != null)
                q = Configs[posCounter < Configs.Count - 1 ? posCounter++ : posCounter];

            shader.Use();

            // joints
            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i].Render(shader, MeshMode.Solid | MeshMode.Wireframe | MeshMode.Lighting, showCollider: _showCollider);
            }

            //// links
            //for (int i = 0; i < Links.Length; i++)
            //{
            //    Links[i].Model.State = Links[i].State.ToMatrix(false);
            //    Links[i].Model.Render(shader, MeshMode.Solid | MeshMode.Wireframe | MeshMode.Lighting);
            //}
        }

        public Manipulator DeepCopy()
        {
            Manipulator manip = (Manipulator)MemberwiseClone();

            manip.Links = Array.ConvertAll(Links, x => x.ShallowCopy());
            manip.Joints = Array.ConvertAll(Joints, x => x.ShallowCopy());

            if (Path != null)
                manip.Path = new List<Vector3>(Path);

            return manip;
        }
    }
}
