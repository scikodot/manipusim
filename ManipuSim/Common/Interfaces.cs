using BulletSharp;

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

        void UpdateState(ref OpenToolkit.Mathematics.Matrix4 state);
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

        ref Vector3 InitialPosition { get; }

        void Translate(Vector3 translation);
    }
}
