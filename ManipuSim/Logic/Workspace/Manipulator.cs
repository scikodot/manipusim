using System;
using System.Collections.Generic;
using System.Linq;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic.InverseKinematics;
using Logic.PathPlanning;
using Physics;

namespace Logic
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

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

        // TODO: goals should be part of Controller tasks, not manipulator attributes
        private Vector3 _goal;
        public ref Vector3 Goal => ref _goal;

        public Path Path;

        public Controller Controller { get; set; }

        public bool ShowCollider { get; set; }

        // TODO: all algorithmic properties should be Controller attributes
        public bool ShowTree { get; set; }

        // TODO: rename Gripper to EndEffector
        public Vector3 GripperPos { get; private set; }  // TODO: create a separate class Gripper/Tool/etc. and probably use DKP[Joints.Length - 1]

        public Vector3[] DKP { get; private set; }  // TODO: rename to JointPositions/DirectKinematics/etc.

        public VectorFloat Coordinates
        {
            get => VectorFloat.Build.Dense(Joints.Select(joint => joint.Coordinate).ToArray());
            set
            {
                for (int i = 0; i < Joints.Length; i++)
                {
                    var (l, u) = Joints[i].CoordinateRange;
                    l *= MathUtil.SIMD_RADS_PER_DEG;
                    u *= MathUtil.SIMD_RADS_PER_DEG;
                    MathUtil.Clamp(value[i], l, u);
                    Joints[i].Coordinate = value[i];
                }
            }
        }

        private Manipulator(Joint[] joints, Link[] links)
        {
            Joints = joints;
            Links = links;

            UpdateRelativeStates();

            WorkspaceRadius = Links.Sum(link => link.Length) + 2 * Joints.Sum(joint => joint.Radius);

            // subscribe to events
            foreach (var joint in Joints)
            {
                joint.TranslationChanged += OnTranslationChanged;
            }

            /*var solver = DampedLeastSquares.Default();
            var planner = GeneticAlgorithm.Default();
            var controller = MotionController.Default();
            manipulator.Controller = new Controller(manipulator, planner, solver, controller);*/
        }

        public static Manipulator CreateDefault(int linksNumber)
        {
            var joints = new Joint[linksNumber + 1];
            var links = new Link[linksNumber];
            for (int i = 0; i < linksNumber; i++)
            {
                joints[i] = new Joint(
                    axis: i == 0 ? Vector3.UnitY : (i % 2 == 0 ? Vector3.UnitZ : Vector3.UnitX), 
                    position: i == 0 ? Vector3.Zero : joints[i - 1].Position + (2 * joints[0].Radius + links[0].Length) * Vector3.UnitY);
                links[i] = new Link();
            }
            joints[^1] = new Joint(
                axis: Vector3.UnitY, 
                position: joints[^2].Position + (2 * joints[0].Radius + links[0].Length) * Vector3.UnitY, 
                isEndEffector: true);

            return new(joints, links);
        }

        public void OnTranslationChanged(object sender, TranslationEventArgs e)
        {
            var joint = sender as Joint;
            int next = Array.IndexOf(Joints, joint) + 1;

            if (next < Joints.Length)
                Joints[next].Translate(e.Translation);
        }

        // TODO: consider performing collision tests through handlers somehow
        // (e.g. ManipulatorHandler queries collisions of its components between each other or with obstacles
        // and probably stores results of those queries somewhere for them to be later used by algorithms)
        public IEnumerable<bool> CollisionTest()
        {
            foreach (var link in Links)
            {
                yield return link.CollisionTest();
            }

            foreach (var joint in Joints)
            {
                yield return joint.Collider.CollisionTest();
            }
        }

        public void Update(InteractionMode mode)
        {
            // update states
            DKP = new Vector3[Joints.Length];
            UpdateJoints();
            UpdateLinks();

            switch (mode)
            {
                case InteractionMode.Design:
                    UpdateRelativeStates();

                    // update links lengths
                    for (int i = 0; i < Links.Length; i++)
                    {
                        var jointDistance = Vector3.Distance(Joints[i + 1].InitialPosition, Joints[i].InitialPosition);
                        jointDistance -= Joints[i + 1].Radius + Joints[i].Radius;

                        (Links[i].Collider as CylinderCollider).HalfLength = jointDistance / 2;
                    }

                    for (int i = 0; i < Joints.Length; i++)
                    {
                        Joints[i].Coordinate = Joints[i].InitialCoordinate;
                    }
                    break;
            }

            // update models
            foreach (var joint in Joints)
                joint.Update(mode);

            foreach (var link in Links)
                link.Update(mode);
        }

        private void UpdateRelativeStates()
        {
            RelativeStates = new ImpDualQuat[Joints.Length];

            ImpDualQuat quat = ImpDualQuat.Zero;
            ImpDualQuat quatRel;

            for (int i = 0; i < Joints.Length; i++)  // TODO: move to DirectKinematics() methods?
            {
                //var offsetAbs = i == 0 ? Joints[i].InitialPosition : Joints[i].InitialPosition - Joints[i - 1].InitialPosition;
                //var offsetRel = i == 0 ? offsetAbs : quat.Conjugate.Rotate(offsetAbs);
                //quatRel = new ImpDualQuat(offsetRel);

                //if (i == 0)
                //    quatRel *= ImpDualQuat.Align(Vector3.UnitY, Joints[i].InitialAxis);
                //else
                //{
                //    // orientation of the joint should not change when the previous joints' axes are changed;
                //    // to maintain this, we have to reset the orientation of the current joint
                //    quatRel *= quat.Conjugate.WithoutTranslation();

                //    // and align the joint with its axis
                //    quatRel *= ImpDualQuat.Align(Vector3.UnitY, Joints[i].InitialAxis);
                //}

                var quatRotAbs = i == 0 ?
                    ImpDualQuat.Align(Vector3.UnitY, Joints[i].InitialAxis) :
                    ImpDualQuat.Align(Joints[i - 1].InitialAxis, Joints[i].InitialAxis);

                var offsetAbs = i == 0 ? Joints[i].InitialPosition : Joints[i].InitialPosition - Joints[i - 1].InitialPosition;
                var quatTransAbs = new ImpDualQuat(offsetAbs);
                var quatAbs = quatTransAbs * quatRotAbs;

                quatRel = quat.Conjugate().WithoutTranslation() * quatAbs * quat.WithoutTranslation();

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
                    (Joints[i].Radius + Links[i].Length / 2) * quat.Conjugate().Rotate(Vector3.Normalize(Joints[i + 1].Position - Joints[i].Position)));

                Links[i].UpdateState(ref quat);
            }
        }

        public void Reset()
        {
            // reset coordinates to their default values
            Coordinates = VectorFloat.Build.Dense(Joints.Select(x => x.InitialCoordinate).ToArray());

            // clear path
            Path = null;
        }

        public bool ApproxWithinReach(Vector3 point)
        {
            return point.Length <= WorkspaceRadius;
        }

        public float DistanceTo(Vector3 p)  // TODO: this one is ambiguous; remove and use usual GripperPos
        {
            return Vector3.Distance(GripperPos, p);
        }

        public void FollowPath()
        {
            if (Path != null)
            {
                Coordinates = Path.Follow().q;
            }
        }

        public void RenderUnselected(ShaderProgram shader)
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

        public void RenderSelected(ShaderProgram shader)
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

        public Manipulator Copy() => new(Array.ConvertAll(Joints, j => j.Copy()), 
                                         Array.ConvertAll(Links, l => l.Copy()));

        public void Dispose()
        {
            foreach (var joint in Joints)
                joint.Dispose();

            foreach (var link in Links)
                link.Dispose();

            Path?.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
