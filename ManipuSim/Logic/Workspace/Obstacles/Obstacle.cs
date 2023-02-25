using System;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic.PathPlanning;
using Physics;

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

        public BroadphaseNativeType Shape => Collider.Shape;

        public Matrix State
        {
            get => Collider.Body.MotionState.WorldTransform;
            set => Collider.Body.MotionState.WorldTransform = value;
        }
        public RigidBodyType Type { get; set; }
        public float Mass { get; set; }
        public bool ShowCollider { get; set; }
        public float Speed { get; } = 0.016f;

        private Vector3 _orientation;
        public Vector3 Orientation
        {
            get => _orientation;
            set
            {
                Rotate(value - _orientation);
                _orientation = value;
            }
        }

        private Vector3 _position;
        public Vector3 Position
        {
            get => _position;
            set
            {
                Translate(value - _position);
                _position = value;
            }
        }

        public Obstacle(Model model, Collider collider)
        {
            Model = model;
            Collider = collider;

            //Path = new Path(new Path.Node(null, new Vector3[] { _position }, null));
            //Path.Model.SetColor(new System.Numerics.Vector3(0.2f, 0.6f, 0.08f));

            _position = Collider.Body.WorldTransform.Origin;

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

            if (!Collider.Body.CollisionFlags.HasFlag(CollisionFlags.KinematicObject))
                throw new InvalidOperationException("Attempt to translate a non-kinematic object.");

            Collider.Body.MotionState.WorldTransform += Matrix.Translation(translation);

            //Path.Translate(translation);
        }

        public void Rotate(Vector3 rotation)
        {
            if (!Collider.Body.CollisionFlags.HasFlag(CollisionFlags.KinematicObject))
                throw new InvalidOperationException("Attempt to rotate a non-kinematic object.");

            var (yaw, pitch, roll) = rotation * MathUtil.SIMD_RADS_PER_DEG;
            Collider.Body.MotionState.WorldTransform = 
                Matrix.RotationYawPitchRoll(yaw, pitch, roll) * Collider.Body.MotionState.WorldTransform;
        }

        public void Render(Shader shader, Action render = null)
        {
            Model.Render(shader, render);

            if (ShowCollider)
                Collider.Render(shader);
        }

        public void Convert(RigidBodyType type, float? mass = null)
        {
            Collider.Convert(type, mass);
        }

        public void Reset()
        {
            var (yaw, pitch, roll) = _orientation * MathUtil.SIMD_RADS_PER_DEG;
            Collider.Body.MotionState.WorldTransform = Matrix.RotationYawPitchRoll(yaw, pitch, roll) * Matrix.Translation(_position);

            Collider.Reset();
            Path.Reset();

            //Collider.Scale();
        }

        public void Update(InteractionMode mode)
        {
            /*if (Collider.Type == RigidBodyType.Kinematic)
                FollowPath();*/

            UpdateModel();
        }

        public void OnInteractionModeSwitched(InteractionModeSwitchEventArgs e)
        {
            switch (e.Mode)
            {
                case InteractionMode.Design:
                    Convert(RigidBodyType.Kinematic, Mass);
                    //Collider.Body.ForceActivationState(ActivationState.DisableSimulation);
                    Reset();
                    break;
                case InteractionMode.Animate:
                    //Collider.Body.ForceActivationState(ActivationState.ActiveTag);
                    Convert(Type, Mass);
                    break;
            }
        }

        /*private void UpdateState()
        {
            // TODO: optimize; consider using ImpDualQuats
            // TODO: create separate method RotateWorld() that will create rotation matrix about world XYZ axes
            var yaw = _orientation.Y * MathUtil.SIMD_RADS_PER_DEG;
            var pitch = _orientation.X * MathUtil.SIMD_RADS_PER_DEG;
            var roll = _orientation.Z * MathUtil.SIMD_RADS_PER_DEG;

            State = Matrix.RotationQuaternion(BulletSharp.Math.Quaternion.RotationYawPitchRoll(yaw, pitch, roll)) *
                Matrix.Translation(_initialPosition.ToBullet3());
        }*/

        private void UpdateModel()
        {
            var state = Matrix.Scaling(Collider.Body.CollisionShape.LocalScaling) * State;
            Model.State = state.ToOpenTK();
            Collider.UpdateModel();
            Path.Model.Update();
        }

        private void FollowPath()
        {
            /*if (Path != null)
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
            }*/
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
