using BulletSharp;
using BulletSharp.Math;

namespace Physics
{
    class SphereCollider : Collider
    {
        public float Radius
        {
            get => _size.X;
            set => Size = new Vector3(value);
        }

        public SphereCollider(PhysicsHandler handler, RigidBody body, RigidBodyType type) : base(handler, body, type)
        {
            var shape = body.CollisionShape as SphereShape;
            _size = new Vector3(shape.Radius);
        }

        public override bool Contains(Vector3 point)
        {
            return Vector3.Distance(Body.CenterOfMassPosition, point) <= Radius;
        }

        public override Vector3 Extrude(Vector3 point)
        {
            var center = Body.CenterOfMassPosition;
            Vector3 vec = point - new Vector3(center.X, center.Y, center.Z);
            return Vector3.Normalize(vec) * Radius - vec;
        }
    }
}
