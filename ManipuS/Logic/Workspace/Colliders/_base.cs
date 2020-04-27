using System.Numerics;

namespace Logic
{
    public class Collider
    {
        public Vector3[] Data;
        public Vector3 Center;
        public Graphics.PlainModel Model;

        protected Collider(Vector3[] data)
        {
            Data = data;
        }

        public virtual bool Contains(Vector3 point)
        {
            return default;
        }

        public virtual Vector3 Extrude(Vector3 point)
        {
            return default;
        }

        public virtual void Render(Graphics.Shader shader, ref Matrix4 state)
        {
            
        }
    }
}