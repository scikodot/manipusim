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

    public class Obstacle : IDisposable
    {
        public Model Model { get; }
        public Collider Collider { get; }

        public BroadphaseNativeType ShapeType => Collider.Shape;
        public ref RigidBodyType Type => ref Collider.Type;

        public Matrix State
        {
            get
            {
                Collider.Body.MotionState.GetWorldTransform(out Matrix state);
                return state;
            }
            set
            {
                Collider.Body.MotionState.SetWorldTransform(ref value);
            }
        }

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        private Vector3 _orientation;
        public ref Vector3 Orientation => ref _orientation;

        private Vector3 _translation;
        public ref Vector3 Translation => ref _translation;

        private Vector3 _scale = Vector3.One;
        public ref Vector3 Scale => ref _scale;

        public Obstacle(Model model, Collider collider)  // TODO: check collider for null; in that case, the obstacle may not participate in collision checks
        {
            Model = model;
            Collider = collider;

            //var trans = Collider.Body.WorldTransform.Row4;
            //_translation = new Vector3(trans.X, trans.Y, trans.Z);

            //var scale = Collider.Body.WorldTransform.ScaleVector;
            //_scale = new Vector3(scale.X, scale.Y, scale.Z);
        }

        public bool Contains(Vector3 point)
        {
            return Collider.Contains(point);
        }

        public Vector3 Extrude(Vector3 point)
        {
            return Collider.Extrude(point);
        }

        public void Move(Vector3 offset)
        {
            State *= Matrix.Translation(offset.X, offset.Y, offset.Z);
        }

        public void Render(Shader shader, MeshMode mode, Action render = null)
        {
            // update obstacle and collider models to reflect object's current state
            UpdateState();

            Model.Render(shader, mode, render);

            if (_showCollider)
                Collider.Render(shader);
        }

        public void Convert(RigidBodyType type)
        {
            Collider.Convert(type);
        }

        public void Reset()
        {

        }

        public void UpdateStateDesign()
        {
            Collider.Scale();

            var degToRad = (float)Math.PI / 180;
            State = 
                Matrix.RotationYawPitchRoll(_orientation.Z * degToRad, _orientation.X * degToRad, _orientation.Y * degToRad) *
                Matrix.Translation(_translation.X, _translation.Y, _translation.Z);
        }

        public void UpdateState()
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
