using BulletSharp;
using BulletSharp.Math;

using Physics;

namespace Graphics
{
    public interface ICollidable
    {
        Model Model { get; }
        RigidBody Body { get; }

        bool Contains(Vector3 point);

        Vector3 Extrude(Vector3 point);

        void Render(Shader shader);

        void UpdateState(ref OpenTK.Mathematics.Matrix4 state);
    }

    public interface ISelectable  // TODO: consider switching to abstract class Selectable
    {
        Model Model { get; }
        Collider Collider { get; }
    }

    public interface ITranslatable
    {
        Model Model { get; }
        Collider Collider { get; }

        Vector3 Position { get; set; }  // TODO: this could have a default impl in an abstract class

        void Translate(Vector3 translation);
    }
}
