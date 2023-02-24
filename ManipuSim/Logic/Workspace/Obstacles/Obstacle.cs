using System;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic.PathPlanning;
using Physics;

using Vector3 = System.Numerics.Vector3;

namespace Logic
{
    /*public struct ObstData
    {
        public float Radius;
        public Vector3 Center;
        public int PointsNum;
        public bool ShowCollider;
    }*/

    public class Obstacle : IDisposable, ISelectable, ITranslatable
    {
        public Model Model { get; }
        public Collider Collider { get; }
        public Path Path { get; }

        public RigidBodyType Type => Collider.Type;
        public BroadphaseNativeType Shape => Collider.Shape;

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

        public Vector3 Position { get; private set; }
        public float Speed { get; } = 0.016f;

        public Obstacle(Model model, Collider collider)
        {
            Model = model;
            Collider = collider;

            Path = new Path(new Path.Node(null, new Vector3[] { _initialPosition }, null));
            Path.Model.SetColor(new Vector3(0.2f, 0.6f, 0.08f));

            _initialPosition = Collider.Body.WorldTransform.Origin.ToNumerics3();

            Model.RenderFlags = RenderFlags.Solid | RenderFlags.Lighting;
            Collider.Body.UserObject = this;
        }

        public bool Contains(Vector3 point) => Collider.Contains(point);

        public Vector3 Extrude(Vector3 point) => Collider.Extrude(point);

        public void Translate(Vector3 translation)
        {
            /*if (MainWindow.Mode == InteractionMode.Design)
                Position = _initialPosition += translation;
            else if (MainWindow.Mode == InteractionMode.Animate)
                Position += translation;

            if (MainWindow.Mode == InteractionMode.Design && translation != Vector3.Zero)
                Path.Translate(translation);*/

            Collider.Body.Translate(translation.ToBullet3());
            Path.Translate(translation);
        }

        public void Render(Shader shader, Action render = null)
        {
            Model.Render(shader, render);

            if (_showCollider)
                Collider.Render(shader);
        }

        public void Convert(RigidBodyType type, float? mass = null)
        {
            Collider.Convert(type, mass);
        }

        public void Reset()
        {
            Collider.Reset();
            Path.Reset();

            Position = _initialPosition;

            Collider.Scale();
            UpdateState();
        }

        public void Update(InteractionMode mode)
        {
            switch (mode)
            {
                case InteractionMode.Design:
                    Collider.Scale();
                    UpdateState();
                    break;
                case InteractionMode.Animate:
                    if (Type == RigidBodyType.Kinematic)
                    {
                        FollowPath();
                        UpdateState();
                    }
                    break;
            }

            UpdateModel();
        }

        public void OnMainWindowModeSwitched(InteractionModeSwitchEventArgs e)
        {
            switch (e.Mode)
            {
                case InteractionMode.Design:
                    Convert(RigidBodyType.Kinematic, Mass);
                    Reset();
                    break;
                case InteractionMode.Animate:
                    Convert(Type, Mass);
                    break;
            }
        }

        private void UpdateState()
        {
            // TODO: optimize; consider using ImpDualQuats
            // TODO: create separate method RotateWorld() that will create rotation matrix about world XYZ axes
            var yaw = _orientation.Y * MathUtil.SIMD_RADS_PER_DEG;
            var pitch = _orientation.X * MathUtil.SIMD_RADS_PER_DEG;
            var roll = _orientation.Z * MathUtil.SIMD_RADS_PER_DEG;

            State = Matrix.RotationQuaternion(BulletSharp.Math.Quaternion.RotationYawPitchRoll(yaw, pitch, roll)) *
                Matrix.Translation(Position.ToBullet3());
        }

        private void UpdateModel()
        {
            var state = Matrix.Scaling(Collider.Body.CollisionShape.LocalScaling) * State;
            Model.State = state.ToOpenTK();
            Collider.UpdateModel();
            Path.Model.Update();
        }

        private void FollowPath()
        {
            if (Path != null)
            {
                var target = Path.Current.Child;
                if (target != null)
                {
                    var direction = target.Points[0] - Position;
                    Vector3 translation;
                    if (direction.Length() <= Speed)
                    {
                        translation = direction;
                        Translate(translation);
                        Path.Follow();
                    }
                    else
                    {
                        translation = Speed * Vector3.Normalize(direction);
                        Translate(translation);
                    }
                }
            }
        }

        public void Dispose()
        {
            // clear managed resources
            Model.Dispose();
            Collider.Dispose();
            Path.Dispose();

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}
