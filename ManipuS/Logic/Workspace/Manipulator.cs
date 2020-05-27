using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Graphics;
using Logic.PathPlanning;
using Physics;

namespace Logic
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

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

    public struct ForwardKinematicsResult
    {
        public Vector3[] JointPositions;
        public Vector3[] JointAxes;
    }

    public class Manipulator : IDisposable, ISelectable
    {
        // manipulator does not have its own model/collider, it just incorporates those from joints/links;
        // hence, the model and collider can be nulled
        public Model Model { get; } = null;
        public Collider Collider { get; } = null;

        public Link[] Links;
        public Joint[] Joints;
        public ImpDualQuat[] RelativeStates;

        public float WorkspaceRadius { get; }

        public Vector3 Base => Joints[0].Position;

        private Vector3 _goal;
        public ref Vector3 Goal => ref _goal;

        public Path Path;

        public MotionController Controller { get; set; }

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        private bool _showTree = true;
        public ref bool ShowTree => ref _showTree;

        // TODO: rename Gripper to EndEffector
        public Vector3 GripperPos { get; private set; }  // TODO: create a separate class Gripper/Tool/etc. and probably use DKP[Joints.Length - 1]

        public Vector3[] DKP { get; private set; }  // TODO: rename to JointPositions/DirectKinematics/etc.

        public VectorFloat q
        {
            get => VectorFloat.Build.Dense(Joints.Select(joint => joint.Coordinate).ToArray());
            set
            {
                for (int i = 0; i < Joints.Length; i++)
                    Joints[i].Coordinate = value[i];

                UpdateStateAnimate();
            }
        }

        public Manipulator(ManipData data)
        {
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

            UpdateRelativeStates();

            //q = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.Dense(Joints.Select(x => x.Coordinate).ToArray());

            //UpdateStateDesign();

            WorkspaceRadius = Links.Sum(link => link.Length) + 2 * Joints.Sum(joint => joint.Radius);
            
            Goal = data.Goal;

            // subscribe to events
            foreach (var joint in Joints)
            {
                joint.TranslationChanged += OnTranslationChanged;
            }
        }

        public void OnTranslationChanged(object sender, TranslationEventArgs e)
        {
            var joint = sender as Joint;
            int next = Array.IndexOf(Joints, joint) + 1;

            if (next < Joints.Length)
                Joints[next].Translate(e.Translation);
        }

        public IEnumerable<bool> CollisionTest()
        {
            foreach (var link in Links)
            {
                yield return link.CollisionTest();
            }

            //foreach (var joint in Joints)
            //{
            //    yield return joint.Collider.CollisionTest();
            //}
        }

        public void UpdateStateDesign()
        {
            DKP = new Vector3[Joints.Length];

            UpdateRelativeStates();
            UpdateJoints();
            UpdateLinks();

            // update links lengths
            for (int i = 0; i < Links.Length; i++)
            {
                var jointDistance = Vector3.Distance(Joints[i + 1].InitialPosition, Joints[i].InitialPosition);
                jointDistance -= Joints[i + 1].Radius + Joints[i].Radius;

                (Links[i].Collider as CylinderCollider).HalfLength = jointDistance / 2;
                Links[i].UpdateStateDesign();
            }

            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i].UpdateStateDesign();

                Joints[i].Coordinate = Joints[i].InitialCoordinate;
            }
        }

        public void UpdateModel()
        {
            // update joints models
            foreach (var joint in Joints)
            {
                joint.UpdateModel();
            }

            // update link models
            foreach (var link in Links)
            {
                link.UpdateModel();
            }
        }

        public void UpdateStateAnimate()
        {
            DKP = new Vector3[Joints.Length];

            UpdateJoints();
            UpdateLinks();
        }

        private void UpdateRelativeStates()
        {
            RelativeStates = new ImpDualQuat[Joints.Length];

            ImpDualQuat quat = ImpDualQuat.Zero;
            ImpDualQuat quatRel;

            for (int i = 0; i < Joints.Length; i++)  // TODO: move to DirectKinematics() methods?
            {
                var offsetAbs = i == 0 ? Joints[i].InitialPosition : Joints[i].InitialPosition - Joints[i - 1].InitialPosition;
                var offsetRel = i == 0 ? offsetAbs : quat.Conjugate.Rotate(offsetAbs);
                quatRel = new ImpDualQuat(offsetRel);

                if (i == 0)
                    quatRel *= ImpDualQuat.Align(Vector3.UnitY, Joints[i].InitialAxis);
                else
                {
                    // orientation of the joint should not change when the previous joints' axes are changed;
                    // to maintain this, we have to reset the orientation of the current joint
                    quatRel *= quat.Conjugate.WithoutTranslation();

                    // and align the joint with its axis
                    quatRel *= ImpDualQuat.Align(Vector3.UnitY, Joints[i].InitialAxis);
                }

                RelativeStates[i] = quatRel;

                // track the current state
                quat *= quatRel;
            }
        }

        public ForwardKinematicsResult ForwardKinematics(VectorFloat coordinates)
        {
            var fkRes = new ForwardKinematicsResult
            {
                JointPositions = new Vector3[Joints.Length],
                JointAxes = new Vector3[Joints.Length],
            };

            ImpDualQuat quat = ImpDualQuat.Zero;
            for (int i = 0; i < Joints.Length; i++)
            {
                // TODO: optimize
                quat *= RelativeStates[i];
                quat *= new ImpDualQuat(Vector3.UnitY, -coordinates[i]);

                fkRes.JointPositions[i] = quat.Translation;
                fkRes.JointAxes[i] = quat.Rotate(Vector3.UnitY);
            }

            return fkRes;
        }

        private void UpdateJoints()
        {
            ImpDualQuat quat = ImpDualQuat.Zero;

            for (int i = 0; i < Joints.Length; i++)  // TODO: move to DirectKinematics() methods?
            {
                // TODO: optimize
                quat *= RelativeStates[i];
                quat *= new ImpDualQuat(Vector3.UnitY, -Joints[i].Coordinate);

                Joints[i].Axis = quat.Rotate(Vector3.UnitY);
                Joints[i].Position = DKP[i] = quat.Translation;

                Joints[i].UpdateState(ref quat);
            }

            GripperPos = DKP[Joints.Length - 1];
        }

        private void UpdateLinks()
        {
            ImpDualQuat quat;

            for (int i = 0; i < Links.Length; i++)
            {
                quat = new ImpDualQuat(Joints[i].Position);

                quat *= ImpDualQuat.Align(Vector3.UnitY, Joints[i + 1].Position - Joints[i].Position);

                quat *= new ImpDualQuat(
                    (Joints[i].Radius + Links[i].Length / 2) * quat.Conjugate.Rotate(Vector3.Normalize(Joints[i + 1].Position - Joints[i].Position)));

                Links[i].UpdateState(ref quat);
            }
        }

        public void Reset()
        {
            // reset GCs to default values
            q = VectorFloat.Build.Dense(Joints.Select(x => x.InitialCoordinate).ToArray());

            // clear path
            Path = null;
        }

        public bool ApproxWithinReach(Vector3 point)
        {
            return point.Length() <= WorkspaceRadius;
        }

        public float DistanceTo(Vector3 p)  // TODO: this one is ambiguous; remove and use usual GripperPos
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

        public void RenderUnselected(Shader shader)
        {
            // joints
            foreach (var joint in Joints)
            {
                if (!joint.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    joint.Render(shader);
            }

            // links
            foreach (var link in Links)
            {
                if (!link.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    link.Render(shader);
            }
        }

        public void RenderSelected(Shader shader)
        {
            // joints
            foreach (var joint in Joints)
            {
                if (joint.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    joint.Render(shader);
            }

            // links
            foreach (var link in Links)
            {
                if (link.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    link.Render(shader);
            }
        }

        public Manipulator DeepCopy()
        {
            Manipulator manip = (Manipulator)MemberwiseClone();

            manip.Links = Array.ConvertAll(Links, x => x.DeepCopy());
            manip.Joints = Array.ConvertAll(Joints, x => x.DeepCopy());

            return manip;
        }

        public void Dispose()
        {
            // clear managed resources
            foreach (var joint in Joints)
            {
                joint.Dispose();
            }

            foreach (var link in Links)
            {
                link.Dispose();
            }

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}
