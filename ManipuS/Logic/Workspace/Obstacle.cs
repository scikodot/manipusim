using System.Numerics;
using Graphics;
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
        public Graphics.ComplexModel Model;

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

        public void Render(Shader shader, bool showCollider = false)
        {
            if (Model == default)
                Model = new ComplexModel(Utils.GLConvert(Data), material: new Assimp.Material
                {
                    ColorAmbient = new Assimp.Color4D(0.0f, 0.0f, 0.0f, 0.0f),
                    ColorDiffuse = new Assimp.Color4D(1.0f, 1.0f, 1.0f, 1.0f)
                });

            var stateMatrix = State.ToMatrix();
            Model.State = stateMatrix;
            Model.Render(shader, MeshMode.Solid, () =>
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
