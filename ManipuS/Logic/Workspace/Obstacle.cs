using System;
using System.Numerics;

using OpenTK.Graphics.OpenGL4;

using Graphics;

namespace Logic
{
    public enum ObstacleShape
    {
        Box,
        Sphere
    }

    public class Obstacle
    {
        private Vector3[] Data;
        public ImpDualQuat State;

        public Model Model;
        public ICollidable Collider;

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

        public void Render(Shader shader, Action render = null, bool showCollider = false)  // TODO: move showCollider to properties
        {
            //if (Model == default)
            //    Model = new Model(MeshVertex.Convert(Data), material: MeshMaterial.White);

            OpenTK.Matrix4 stateMatrix = State.ToMatrix();
            Model.State = stateMatrix;
            Model.Render(shader, MeshMode.Solid | MeshMode.Lighting, render);

            if (showCollider)
            {
                Collider.Render(shader, ref stateMatrix);
            }
        }

        private void UpdateState()
        {
            OpenTK.Matrix4 stateMatrix = State.ToMatrix();
            Model.State = stateMatrix;
            Collider.UpdateState(ref stateMatrix);
        }
    }
}
