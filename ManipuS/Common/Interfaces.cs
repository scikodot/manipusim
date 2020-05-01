using BulletSharp;
using OpenTK;

using Vector3 = System.Numerics.Vector3;

namespace Graphics
{
    public interface IRenderable
    {
        ref Matrix4 State { get; }
    }

    public interface ICollidable
    {
        Model Model { get; }
        RigidBody Body { get; }

        bool Contains(Vector3 point);

        Vector3 Extrude(Vector3 point);

        void Render(Shader shader, ref Matrix4 state);

        void UpdateState(ref Matrix4 state);
    }
}
