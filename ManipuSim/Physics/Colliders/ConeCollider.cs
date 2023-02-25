using System;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic;

namespace Physics
{
    class ConeCollider : Collider
    {
        private float _radius;
        public ref float Radius => ref _radius;

        private float _height;
        public ref float Height => ref _height;

        public ConeCollider(PhysicsHandler handler, RigidBody body, RigidBodyType type) : base(handler, body, type)
        {
            var shape = body.CollisionShape as ConeShape;
            _radius = shape.Radius;
            _height = shape.Height;
        }

        public override void Scale()
        {
            var shape = (ConeShape)Body.CollisionShape;
            var radiusRatio = _radius / shape.Radius;
            var heightRatio = _height / shape.Height;
            Body.CollisionShape.LocalScaling *= new Vector3(radiusRatio, heightRatio, radiusRatio);
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
