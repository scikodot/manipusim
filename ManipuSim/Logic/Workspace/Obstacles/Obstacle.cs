using System;
using System.Drawing;
using BulletSharp;
using BulletSharp.Math;
using BulletSharp.SoftBody;
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

    public class Obstacle : IDisposable, ISelectable, IScalable, IRotatable, ITranslatable
    {
        public Model Model { get; }
        public Collider Collider { get; }
        public Path Path { get; }

        public BroadphaseNativeType Shape => Collider.Shape;
        public Matrix State => Collider.State;

        public RigidBodyType Type { get; set; }
        public float Mass { get; set; }
        public bool ShowCollider { get; set; }
        public float Speed { get; } = 0.016f;

        public Vector3 Size
        {
            get => Collider.Size;
            set => Collider.Size = value;
        }

        public Vector3 Orientation
        {
            get => Collider.Orientation;
            set => Collider.Orientation = value;
        }

        public Vector3 Position
        {
            get => Collider.Position;
            set => Collider.Position = value;
        }

        public Obstacle(Model model, Collider collider)
        {
            Model = model;
            Collider = collider;

            //Path = new Path(new Path.Node(null, new Vector3[] { _position }, null));
            //Path.Model.SetColor(new System.Numerics.Vector3(0.2f, 0.6f, 0.08f));

            Model.RenderFlags = RenderFlags.Default | RenderFlags.Lighting;
            Collider.Body.UserObject = this;
        }

        public bool Contains(Vector3 point) => Collider.Contains(point);

        public Vector3 Extrude(Vector3 point) => Collider.Extrude(point);

        public void Scale(Vector3 scaling) => Collider.Scale(scaling);

        public void Rotate(Vector3 rotation) => Collider.Rotate(rotation);

        public void Translate(Vector3 translation) => Collider.Translate(translation);

        public void Update(InteractionMode mode)
        {
            /*if (Collider.Type == RigidBodyType.Kinematic)
                FollowPath();*/

            var state = State.ToOpenTK();
            Model.Update(state);
            Collider.Model.Update(state);

        }

        public void Render(ShaderProgram shader)
        {
            Model.Render(shader);

            if (ShowCollider)
                Collider.Render(shader);
        }

        public void Reset()
        {
            Collider.Reset();
            //Path.Reset();
        }

        public void OnInteractionModeSwitched(InteractionModeSwitchEventArgs e)
        {
            switch (e.Mode)
            {
                case InteractionMode.Design:
                    // during design, objects positions are updated manually, 
                    // and only kinematic objects provide this capability
                    Collider.Convert(RigidBodyType.Kinematic, Mass);

                    // interactable objects should not get deactivated, 
                    // otherwise the body position will not be updating
                    Collider.Body.ForceActivationState(ActivationState.DisableDeactivation);

                    // revert body changes that took place during simulation
                    Reset();
                    break;
                case InteractionMode.Simulate:
                    // during simulation, objects positions are updated automatically 
                    // by the physics engine, hence set the target props
                    Collider.Convert(Type, Mass);

                    // here activation state has to be forced; see source for btCollisionObject::activate
                    Collider.Body.ForceActivationState(ActivationState.ActiveTag);

                    /* When object's velocities (both linear and angular) fall below respective thresholds,
                     * the DeactivationTime counter (it is *not* a limit) starts incrementing until it reaches the standard 
                     * time limit, which is 2 seconds. Then, its activation state becomes WantsDeactivation,
                     * and if its island neighbours come to rest, its activation state becomes IslandSleeping.
                     * 
                     * Note, however, that a constant interaction between a dynamic object and a kinematic object 
                     * (as in the first resting on the second) will *not* let the former become deactivated 
                     * (DeactivationTime won't increment), even if it is completely still.
                     * This is reasonable though, because supposedly a kinematic object can be moved at any moment of time.
                     */
                    // reset the deactivation timer (same as in btCollisionObject::activate);
                    Collider.Body.DeactivationTime = 0;
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

        /*private void FollowPath()
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
        }*/

        public void Dispose()
        {
            // clear managed resources
            Model.Dispose();
            Collider.Dispose();
            //Path.Dispose();

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}
