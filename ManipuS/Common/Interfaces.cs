using BulletSharp;
using OpenTK;

using Physics;

using Vector3 = System.Numerics.Vector3;

namespace Graphics
{
    public interface ICollidable
    {
        Model Model { get; }
        RigidBody Body { get; }

        bool Contains(Vector3 point);

        Vector3 Extrude(Vector3 point);

        void Render(Shader shader);

        void UpdateState(ref Matrix4 state);
    }

    public interface ISelectable
    {
        Model Model { get; }
        Collider Collider { get; }
    }
}
