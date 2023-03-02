using BulletSharp;
using BulletSharp.Math;

namespace Physics
{
    class CylinderCollider : Collider
    {
        public float Radius
        {
            get => _size.X;
            set => Size = new Vector3(value, _size.Y, value);
        }

        public float HalfLength
        {
            get => _size.Y;
            set => Size = new Vector3(_size.X, value, _size.Z);
        }

        public CylinderCollider(PhysicsHandler handler, RigidBody body, RigidBodyType type) : base(handler, body, type)
        {
            var shape = body.CollisionShape as CylinderShape;
            _size = new Vector3(shape.Radius, shape.HalfExtentsWithMargin.Y, shape.Radius);
        }

        public override bool Contains(Vector3 point)
        {
            // TODO: implement
            return default;
        }

        public override Vector3 Extrude(Vector3 point)
        {
            // TODO: implement
            return default;
        }
    }
}
