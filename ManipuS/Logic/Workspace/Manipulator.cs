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

        private Vector3 _goal;
        public ref Vector3 Goal => ref _goal;

        public Path Path;
        public Tree Tree;
        public List<Attractor> Attractors;

        public MotionController Controller { get; set; }

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        private bool _showTree = true;
        public ref bool ShowTree => ref _showTree;

        public bool IsOriginal { get; private set; }

        public Vector3 GripperPos { get; private set; }  // TODO: create a separate class Gripper/Tool/etc.

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

        public Manipulator(ManipData data)
        {
            Base = data.JointPositions[0];
            Links = Array.ConvertAll(data.Links, x => new Link(x));
            Joints = Array.ConvertAll(data.Joints, x => new Joint(x));

            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i].InitialAxis = data.JointAxes[i];
                Joints[i].InitialPosition = data.JointPositions[i];
            }

            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i].Axis = data.JointAxes[i];
                Joints[i].Position = data.JointPositions[i];
            }

            _q = new Vector(Joints.Select(x => x.q).ToArray());

            IsOriginal = true;

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
                    quat *= new ImpDualQuat(Joints[j + 1].InitialPosition - Joints[j].InitialPosition);  // TODO: this *may* cause inappropriate behaviour; consider initPos[j + 1] - initPos[i]
                    quat *= new ImpDualQuat(quat.Conjugate, Joints[j].InitialAxis, quat.Translation, Joints[j].Position, -Joints[j].q);  // TODO: optimize; probably, conjugation can be avoided
                }

                quat *= new ImpDualQuat(Joints[i].InitialAxis, -Joints[i].q);

                quat *= ImpDualQuat.Align(Joints[0].Axis, Joints[i].InitialAxis);

                // update physics and graphics only if the current manipulator is the real one
                if (IsOriginal)
                    Joints[i].UpdateState(ref quat);

                Joints[i].Axis = quat.Rotate(Joints[0].Axis);
                Joints[i].Position = DKP[i] = quat.Translation;
            }

            // links
            if (IsOriginal)
            {
                for (int i = 0; i < Links.Length; i++)
                {
                    quat = init;

                    for (int j = 0; j <= i; j++)
                    {
                        quat *= j == 0 ?
                            new ImpDualQuat(Joints[0].Length / 2 * Vector3.UnitY) :
                            new ImpDualQuat(Joints[j].InitialPosition - Joints[j - 1].InitialPosition);
                        quat *= new ImpDualQuat(quat.Conjugate, Joints[j].InitialAxis, quat.Translation, Joints[j].Position, -Joints[j].q);
                    }

                    Links[i].UpdateState(ref quat);
                }
            }

            // gripper
            GripperPos = DKP[Joints.Length - 1];
        }

        public void Reset()
        {
            // reset GCs to default values
            q = new Vector(Joints.Select(x => x.InitialCoordinate).ToArray());

            // clear tree and path
            Tree = null;
            Path = null;
        }

        public bool InWorkspace(Vector3 point)  // TODO: not used anywhere; fix
        {
            return point.DistanceTo(Vector3.Zero) - point.DistanceTo(Vector3.Zero) <= WorkspaceRadius;
        }

        public float DistanceTo(Vector3 p)
        {
            return GripperPos.DistanceTo(p);
        }

        public void FollowPath()
        {
            if (Path != null)
            {
                q = Path.Follow();
            }
        }

        public void Render(Shader shader)
        {
            shader.Use();

            // joints
            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i].Render(shader);
            }

            // links
            for (int i = 0; i < Links.Length; i++)
            {
                Links[i].Render(shader, showCollider: _showCollider);
            }
        }

        public Manipulator DeepCopy()
        {
            Manipulator manip = (Manipulator)MemberwiseClone();

            manip.Links = Array.ConvertAll(Links, x => x.ShallowCopy());
            manip.Joints = Array.ConvertAll(Joints, x => x.ShallowCopy());

            //if (Path != null)
            //    manip.Path = new List<Vector3>(Path);

            manip.IsOriginal = false;

            // TODO: clone colliders!

            return manip;
        }
    }
}
