using System.Numerics;

using OpenTK.Graphics.OpenGL4;

namespace Logic
{
    public enum ColliderShape
    {
        Box,
        Sphere
    }

    public class Obstacle
    {
        private Vector3[] Data;
        public ImpDualQuat State;
        public Collider Collider;
        public Graphics.PlainModel Model;

        public Obstacle(Vector3[] data, ImpDualQuat state, ColliderShape shape)
        {
            Data = data;
            State = state;
            
            switch (shape)
            {
                case ColliderShape.Box:
                    Collider = new BoxCollider(Data);
                    break;
                case ColliderShape.Sphere:
                    Collider = new SphereCollider(Data);
                    break;
            }
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
            Collider.Center = State.Translation;
        }

        public void Render(Graphics.Shader shader, bool showCollider = false)
        {
            if (Model == default)
                Model = new Graphics.PlainModel(shader, Graphics.Utils.GL_Convert(Data, OpenTK.Graphics.Color4.White));

            var stateMatrix = State.ToMatrix();
            Model.State = stateMatrix;
            Model.Render(() =>
            {
                GL.DrawArrays(PrimitiveType.Points, 0, Data.Length);
            });

            if (showCollider)
            {
                Collider.Render(shader, ref stateMatrix);
            }
        }
    }
}
