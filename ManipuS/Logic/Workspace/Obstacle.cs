using System;
using System.Numerics;

using Graphics;

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
        public ImpDualQuat State;

        public Model Model { get; }
        public ICollidable Collider { get; }

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        public Obstacle(Model model, ICollidable collider, ImpDualQuat state)  // TODO: check collider for null; in that case, the obstacle may not participate in collision checks
        {
            Model = model;
            Collider = collider;

            State = state;

            UpdateState();
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
            State *= new ImpDualQuat(offset);

            UpdateState();
        }

        public void Render(Shader shader, MeshMode mode, Action render = null)  // TODO: move showCollider to properties
        {
            Model.Render(shader, mode, render);

            if (_showCollider)
                Collider.Render(shader);
        }

        private void UpdateState()
        {
            OpenTK.Matrix4 stateMatrix = State.ToMatrix();
            Model.State = stateMatrix;
            Collider.UpdateState(ref stateMatrix);
        }
    }
}
