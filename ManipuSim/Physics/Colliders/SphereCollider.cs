using System;

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

        public SphereCollider(RigidBody body)
        {
            var shape = (SphereShape)body.CollisionShape;

            if (shape == null)
                throw new ArgumentException("The body shape does not match the collider!", "body.CollisionShape");

            Model = Primitives.Sphere(shape.Radius, 20, 20, MeshMaterial.Green);
            Body = body;

            _radius = shape.Radius;
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
