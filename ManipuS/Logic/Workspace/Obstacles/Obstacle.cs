using System;
using System.Numerics;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
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

        public BroadphaseNativeType ShapeType => Collider.Shape;
        public ref RigidBodyType Type => ref Collider.Type;

        public Matrix State
        {
            get => Collider.Body.MotionState.WorldTransform;
            set => Collider.Body.MotionState.WorldTransform = value;
        }

        //public Matrix State  // TODO: change to WorldTransform
        //{
        //    get
        //    {
        //        Collider.Body.MotionState.GetWorldTransform(out Matrix state);
        //        return state;
        //    }
        //    set
        //    {
        //        Collider.Body.MotionState.SetWorldTransform(ref value);
        //    }
        //}

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        private Vector3 _orientation;
        public ref Vector3 Orientation => ref _orientation;

        private Vector3 _initialPosition;
        public ref Vector3 InitialPosition => ref _initialPosition;

        public Obstacle(Model model, Collider collider)  // TODO: check collider for null; in that case, the obstacle may not participate in collision checks
        {
            Model = model;
            Collider = collider;

            ObstacleHandler.Add(this);  // TODO: perhaps remove and use ObstacleHandler.Add(new Obstacle(...)) ?

            _initialPosition = Collider.Body.WorldTransform.Origin.ToNumerics3();

            Model.RenderFlags = RenderFlags.Solid | RenderFlags.Lighting;
            Collider.Body.UserObject = this;

            //var orientation = Collider.Body.Orientation.  /*Collider.Body.Orientation.Angle * BulletSharp.Math.Vector3.Normalize(Collider.Body.Orientation.Axis);*/
            //_orientation = new Vector3(orientation.X, orientation.Y, orientation.Z) * MathUtil.SIMD_DEGS_PER_RAD;
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
            // update obstacle and collider models to reflect object's current state
            UpdateState();

            Model.Render(shader, render);

            if (_showCollider)
                Collider.Render(shader);
        }

        public void Convert(RigidBodyType type)
        {
            Collider.Convert(type);
        }

        public void Reset()
        {
            UpdateStateDesign();
        }

        public void UpdateStateDesign()
        {
            Collider.Scale();

            var inverse = Collider.Body.Orientation.Inverse;

            // TODO: optimize; consider using ImpDualQuats
            // TODO: create separate method RotateWorld() that will create rotation matrix about world XYZ axes
            var matX = Matrix.RotationAxis(inverse.Rotate(BulletSharp.Math.Vector3.UnitX), _orientation.X * MathUtil.SIMD_RADS_PER_DEG);
            var matY = Matrix.RotationAxis(inverse.Rotate(BulletSharp.Math.Vector3.UnitY), _orientation.Y * MathUtil.SIMD_RADS_PER_DEG);
            var matZ = Matrix.RotationAxis(inverse.Rotate(BulletSharp.Math.Vector3.UnitZ), _orientation.Z * MathUtil.SIMD_RADS_PER_DEG);

            State = matX * matY * matZ * Matrix.Translation(_initialPosition.ToBullet3());
        }

        public void UpdateState()  // TODO: move to UpdateFrame!
        {
            var state = Matrix.Scaling(Collider.Body.CollisionShape.LocalScaling) * State;

            OpenTK.Matrix4 stateMatrix = new OpenTK.Matrix4(
                state.M11, state.M21, state.M31, state.M41,
                state.M12, state.M22, state.M32, state.M42,
                state.M13, state.M23, state.M33, state.M43,
                state.M14, state.M24, state.M34, state.M44);

            Model.State = stateMatrix;
            Collider.UpdateState(ref stateMatrix);
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
