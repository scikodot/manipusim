using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK;
using OpenTK.Graphics.OpenGL4;
using Graphics;
using Logic.PathPlanning;

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
        public Entity Entity;

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

        public void Draw(Shader shader, bool showCollider = false)
        {
            if (Entity == null)
                Entity = new Entity(shader, Utils.GL_Convert(Data, Vector4.One));

            Matrix4 model = State.ToMatrix(true);
            Entity.Display(model, () =>
            {
                GL.DrawArrays(PrimitiveType.Points, 0, Data.Length);
            });

            if (showCollider)
                Collider.Draw(shader, model);
        }
    }
}
