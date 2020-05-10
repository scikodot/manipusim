using System;

using BulletSharp;

using Graphics;
using Logic;

using Vector3 = System.Numerics.Vector3;

namespace Physics
{
    class CylinderCollider : Collider
    {
        private float _radius;
        public ref float Radius => ref _radius;

        private float _halfLength;
        public ref float HalfLength => ref _halfLength;

        public CylinderCollider(RigidBody body)
        {
            var shape = (CylinderShape)body.CollisionShape;

            if (shape == null)
                throw new ArgumentException("The body shape does not match the collider!", "body.CollisionShape");

            Model = Primitives.Cylinder(shape.Radius, shape.HalfExtentsWithMargin.Y, shape.HalfExtentsWithMargin.Y, 20, MeshMaterial.Green);
            Body = body;

            _radius = shape.Radius;
            _halfLength = shape.HalfExtentsWithMargin.Y;
        }

        public override void Scale()
        {
            Body.CollisionShape.LocalScaling *= new BulletSharp.Math.Vector3(_radius, _halfLength, _radius) / ((CylinderShape)Body.CollisionShape).HalfExtentsWithMargin;
        }

        public override bool Contains(Vector3 point)
        {
            return default;
        }

        public override Vector3 Extrude(Vector3 point)
        {
            return default;
        }
    }
}
