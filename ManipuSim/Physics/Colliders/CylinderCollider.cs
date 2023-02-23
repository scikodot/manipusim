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

        public CylinderCollider(RigidBody body, RigidBodyType type) : base(body, type)
        {
            if (body.CollisionShape is not CylinderShape shape)
                throw new ArgumentException($"Expected {nameof(CylinderShape)} collision shape; got {body.CollisionShape.GetType().Name}.");

            _radius = shape.Radius;
            _halfLength = shape.HalfExtentsWithMargin.Y;

            // TODO: why does Primitives.Cylinder return a Mesh instead of a Model?
            Model = new Model(new Mesh[]
            {
                Primitives.Cylinder(_radius, _halfLength, _halfLength, 20, MeshMaterial.Green)
            });
            Body = body;
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
