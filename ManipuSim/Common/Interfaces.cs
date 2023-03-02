using BulletSharp;
using BulletSharp.Math;

using Physics;

namespace Graphics
{
    /*public interface ICollidable
    {
        Model Model { get; }
        RigidBody Body { get; }

        bool Contains(Vector3 point);

        Vector3 Extrude(Vector3 point);

        void Render(Shader shader);

        void UpdateState(ref OpenTK.Mathematics.Matrix4 state);
    }*/

    public interface ISelectable  // TODO: consider switching to abstract class Selectable
    {
        Model Model { get; }
        Collider Collider { get; }
    }

    public interface IScalable
    {
        Model Model { get; }
        Collider Collider { get; }

        Vector3 Size { get; set; }

        void Scale(Vector3 scaling);
    }

    public interface IRotatable
    {
        Model Model { get; }
        Collider Collider { get; }

        Vector3 Orientation { get; set; }

        void Rotate(Vector3 rotation);
    }

    public interface ITranslatable
    {
        Model Model { get; }
        Collider Collider { get; }

        Vector3 Position { get; set; }

        void Translate(Vector3 translation);
    }
}
