using System;
using System.Numerics;

using Graphics;

namespace Logic
{
    public enum ObstacleShape
    {
        Box,
        Sphere
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

        //public Obstacle(Vector3[] data, ImpDualQuat state, ColliderShape shape)
        //{
        //    Data = data;
        //    State = state;
            
        //    switch (shape)
        //    {
        //        case ColliderShape.Box:
        //            Collider = new BoxCollider(Data);
        //            break;
        //        case ColliderShape.Sphere:
        //            Collider = new SphereCollider(Data);
        //            break;
        //    }
        //}

        public Obstacle(Model model, ICollidable collider, ImpDualQuat state)
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
