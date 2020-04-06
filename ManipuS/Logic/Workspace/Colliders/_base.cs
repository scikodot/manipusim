using Graphics;

namespace Logic
{
    public class Collider
    {
        public Vector3[] Data;
        public Vector3 Center;
        public Entity Entity;

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

        public virtual void Draw(Shader shader, Matrix4 model)
        {
            
        }
    }
}