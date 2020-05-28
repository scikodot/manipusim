using System;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic.PathPlanning;
using Physics;

using Vector3 = System.Numerics.Vector3;

namespace Logic
{
    public struct ObstData
    {
        public float Radius;
        public Vector3 Center;
        public int PointsNum;
        public bool ShowCollider;
    }

    public class Obstacle : IDisposable, ISelectable, ITranslatable
    {
        public Model Model { get; }
        public Collider Collider { get; }

        public Path Path { get; }

        public BroadphaseNativeType Shape => Collider.Shape;
        public RigidBodyType Type
        {
            get => Collider.Type;
            set => Collider.Type = value;
        }

        public Matrix State
        {
            get => Collider.Body.WorldTransform;
            set
            {
                // explicitly set position of the body
                Collider.Body.WorldTransform = value;

                // set its motion state to update position (for kinematic objects only)
                if (Collider.Body.CollisionFlags.HasFlag(CollisionFlags.KinematicObject))
                    Collider.Body.MotionState.SetWorldTransform(ref value);
            }
        }

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        private float _mass;
        public ref float Mass => ref _mass;

        private Vector3 _orientation;
        public ref Vector3 Orientation => ref _orientation;

        private Vector3 _initialPosition;
        public ref Vector3 InitialPosition => ref _initialPosition;

        public Obstacle(Model model, Collider collider)  // TODO: check collider for null; in that case, the obstacle may not participate in collision checks
        {
            Model = model;
            Collider = collider;

            Path = new Path(new Path.Node(null, new Vector3[] { _initialPosition }, null));

            _initialPosition = Collider.Body.WorldTransform.Origin.ToNumerics3();

            Model.RenderFlags = RenderFlags.Solid | RenderFlags.Lighting;
            Collider.Body.UserObject = this;
        }

        public bool Contains(Vector3 point)
        {
            return Collider.Contains(point);
        }

        public Vector3 Extrude(Vector3 point)
        {
            return Collider.Extrude(point);
        }

        public void Translate(Vector3 translation)
        {
            _initialPosition += translation;
        }

        public void Render(Shader shader, Action render = null)
        {
            Model.Render(shader, render);

            if (_showCollider)
                Collider.Render(shader);
        }

        public void Convert(RigidBodyType type, float mass)
        {
            Collider.Convert(type, mass);
        }

        public void Reset()
        {
            Collider.Reset();

            UpdateStateDesign();
        }

        public void UpdateStateDesign()
        {
            Collider.Scale();

            // TODO: optimize; consider using ImpDualQuats
            // TODO: create separate method RotateWorld() that will create rotation matrix about world XYZ axes
            var yaw = _orientation.Y * MathUtil.SIMD_RADS_PER_DEG;
            var pitch = _orientation.X * MathUtil.SIMD_RADS_PER_DEG;
            var roll = _orientation.Z * MathUtil.SIMD_RADS_PER_DEG;

            State = Matrix.RotationQuaternion(BulletSharp.Math.Quaternion.RotationYawPitchRoll(yaw, pitch, roll)) * 
                Matrix.Translation(_initialPosition.ToBullet3());
        }

        public void UpdateModel()
        {
            var state = Matrix.Scaling(Collider.Body.CollisionShape.LocalScaling) * State;

            OpenTK.Matrix4 stateMatrix = new OpenTK.Matrix4(
                state.M11, state.M21, state.M31, state.M41,
                state.M12, state.M22, state.M32, state.M42,
                state.M13, state.M23, state.M33, state.M43,
                state.M14, state.M24, state.M34, state.M44);

            Model.State = stateMatrix;
            Collider.UpdateModel(ref stateMatrix);
        }

        public void Dispose()
        {
            // clear managed resources
            Model.Dispose();
            Collider.Dispose();

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}
