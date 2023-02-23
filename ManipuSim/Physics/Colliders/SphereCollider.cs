using System;
using System.Data;
using BulletSharp;

using Graphics;
using Logic;

using Vector3 = System.Numerics.Vector3;

namespace Physics
{
    class SphereCollider : Collider
    {
        private float _radius;
        public ref float Radius => ref _radius;

        public SphereCollider(RigidBody body, RigidBodyType type) : base(body, type)
        {
            if (body.CollisionShape is not SphereShape shape)
                throw new ArgumentException($"Expected {nameof(SphereShape)} collision shape; got {body.CollisionShape.GetType().Name}.");

            _radius = shape.Radius;

            Model = Primitives.Sphere(_radius, 20, 20, MeshMaterial.Green);
            Body = body;
        }

        public override void Scale()
        {
            Body.CollisionShape.LocalScaling *= (Radius / ((SphereShape)Body.CollisionShape).Radius) * BulletSharp.Math.Vector3.One;
        }

        public override bool Contains(Vector3 point)
        {
            var center = Body.CenterOfMassPosition;
            return new Vector3(center.X, center.Y, center.Z).DistanceTo(point) <= _radius;
        }

        public override Vector3 Extrude(Vector3 point)
        {
            var center = Body.CenterOfMassPosition;
            Vector3 vec = point - new Vector3(center.X, center.Y, center.Z);
            return Vector3.Normalize(vec) * _radius - vec;
        }
    }
}
