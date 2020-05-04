using System;
using System.Numerics;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Physics;
using Vector3 = System.Numerics.Vector3;

namespace Logic
{
    public enum PhysicsType
    {
        Static,
        Kinematic,
        Dynamic
    }

    public struct ObstData
    {
        public float Radius;
        public Vector3 Center;
        public int PointsNum;
        public bool ShowCollider;
    }

    public class Obstacle
    {
        public Model Model { get; }
        public Collider Collider { get; }

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

        public Obstacle(Model model, Collider collider)  // TODO: check collider for null; in that case, the obstacle may not participate in collision checks
        {
            Model = model;
            Collider = collider;
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
            Matrix.Translation(offset.X, offset.Y, offset.Z, out Matrix state);
            State *= state;
        }

        public void Render(Shader shader, MeshMode mode, Action render = null)
        {
            // update obstacle and collider models to reflect object's current state
            UpdateState();

            Model.Render(shader, mode, render);

            if (_showCollider)
                Collider.Render(shader);
        }

        public void UpdateState()
        {
            var state = State;
            OpenTK.Matrix4 stateMatrix = new OpenTK.Matrix4(
                state.M11, state.M21, state.M31, state.M41,
                state.M12, state.M22, state.M32, state.M42,
                state.M13, state.M23, state.M33, state.M43,
                state.M14, state.M24, state.M34, state.M44);

            Model.State = stateMatrix;
            Collider.UpdateState(ref stateMatrix);
        }
    }
}
