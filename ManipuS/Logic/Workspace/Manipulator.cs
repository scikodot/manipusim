using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Graphics;
using Logic.PathPlanning;
using Physics;

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

    public class Manipulator : IDisposable, ISelectable
    {
        // manipulator does not have its own model/collider, it just incorporates those from joints/links;
        // hence, the model and collider can be nulled
        public Model Model { get; } = null;
        public Collider Collider { get; } = null;

        public Link[] Links;
        public Joint[] Joints;
        public float WorkspaceRadius { get; }

        public Vector3 Base => Joints[0].Position;

        private Vector3 _goal;
        public ref Vector3 Goal => ref _goal;

        public Path Path;
        public Tree Tree;  // TODO: move to RRT planner!
        public List<Attractor> Attractors;  // TODO: move to RRT planner!

        public MotionController Controller { get; set; }

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        private bool _showTree = true;
        public ref bool ShowTree => ref _showTree;

        public Vector3 GripperPos { get; private set; }  // TODO: create a separate class Gripper/Tool/etc. and probably use DKP[Joints.Length - 1]

        public Vector3[] DKP { get; private set; }  // TODO: rename to JointPositions/DirectKinematics/etc.

        public MathNet.Numerics.LinearAlgebra.Vector<float> q
        {
            get => MathNet.Numerics.LinearAlgebra.Vector<float>.Build.Dense(Joints.Select(joint => joint.Coordinate).ToArray());
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

            q = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.Dense(Joints.Select(x => x.Coordinate).ToArray());

            UpdateStateDesign();

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

            UpdateJoints();
            UpdateGripper();
            UpdateLinks();

            // update links lengths
            for (int i = 0; i < Links.Length; i++)
            {
                var jointDistance = Vector3.Distance(Joints[i + 1].InitialPosition, Joints[i].InitialPosition);
                jointDistance -= Joints[i + 1].Radius + Joints[i].Radius;

                (Links[i].Collider as CylinderCollider).HalfLength = jointDistance / 2;
                Links[i].UpdateStateDesign();
            }

            // TODO: add joints scaling!
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
            UpdateGripper();
            UpdateLinks();
        }

        private void UpdateJoints()
        {
            ImpDualQuat quat;
            ImpDualQuat init = new ImpDualQuat(Joints[0].InitialPosition);

            for (int i = 0; i < Joints.Length - 1; i++)  // TODO: move to DirectKinematics() methods?
            {
                quat = init;

                for (int j = 0; j < i; j++)
                {
                    quat *= new ImpDualQuat(Joints[j + 1].InitialPosition - Joints[j].InitialPosition);  // TODO: this *may* cause inappropriate behaviour; consider initPos[j + 1] - initPos[i]
                    quat *= new ImpDualQuat(quat.Conjugate, Joints[j].InitialAxis, quat.Translation, Joints[j].Position, -Joints[j].Coordinate);  // TODO: optimize; probably, conjugation can be avoided
                }

                quat *= new ImpDualQuat(Joints[i].InitialAxis, -Joints[i].Coordinate);

                // TODO: move to documentation!
                // models initially (on creation) have a specific actuation axis;
                // for example, the two models used for joint and links in this program have their actuation axes UnitY = (0, 1, 0);
                // thus, the models have to be aligned from that state with their InitialAxis axes that are set from the GUI or obtained from code
                quat *= ImpDualQuat.Align(Vector3.UnitY, Joints[i].InitialAxis);

                Joints[i].Axis = quat.Rotate(Vector3.UnitY);
                Joints[i].Position = DKP[i] = quat.Translation;

                Joints[i].UpdateState(ref quat);
            }
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

        private void UpdateGripper()
        {
            ImpDualQuat quat = new ImpDualQuat(Joints[0].InitialPosition);

            var last = Joints.Length - 1;
            for (int j = 0; j < last; j++)
            {
                quat *= new ImpDualQuat(Joints[j + 1].InitialPosition - Joints[j].InitialPosition);  // TODO: this *may* cause inappropriate behaviour; consider initPos[j + 1] - initPos[i]
                quat *= new ImpDualQuat(quat.Conjugate, Joints[j].InitialAxis, quat.Translation, Joints[j].Position, -Joints[j].Coordinate);  // TODO: optimize; probably, conjugation can be avoided
            }

            quat *= ImpDualQuat.Align(Vector3.UnitY, quat.Conjugate.Rotate(Joints[last].Position - Joints[last - 1].Position));
            quat *= new ImpDualQuat(Joints[last].InitialAxis, -Joints[last].Coordinate);
            quat *= ImpDualQuat.Align(Vector3.UnitY, Joints[last].InitialAxis);

            Joints[last].Axis = quat.Rotate(Vector3.UnitY);
            Joints[last].Position = DKP[last] = quat.Translation;

            GripperPos = DKP[Joints.Length - 1];

            Joints[last].UpdateState(ref quat);
        }

        public void Reset()
        {
            // reset GCs to default values
            q = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.Dense(Joints.Select(x => x.InitialCoordinate).ToArray());

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
