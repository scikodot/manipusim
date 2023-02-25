using System;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic;

namespace Physics
{
    class CylinderCollider : Collider
    {
        private float _radius;
        public ref float Radius => ref _radius;

        private float _halfLength;
        public ref float HalfLength => ref _halfLength;

        public CylinderCollider(PhysicsHandler handler, RigidBody body, RigidBodyType type) : base(handler, body, type)
        {
            var shape = body.CollisionShape as CylinderShape;
            _radius = shape.Radius;
            _halfLength = shape.HalfExtentsWithMargin.Y;
        }

        public override void Scale()
        {
            Body.CollisionShape.LocalScaling *= new BulletSharp.Math.Vector3(_radius, _halfLength, _radius) / ((CylinderShape)Body.CollisionShape).HalfExtentsWithMargin;
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
