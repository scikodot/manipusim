using System;
using System.Numerics;

using BulletSharp;

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

        public ConeCollider(RigidBody body)
        {
            var shape = (ConeShape)body.CollisionShape;

            if (shape == null)
                throw new ArgumentException("The body shape does not match the collider!", "body.CollisionShape");

            Model = new Model(new Mesh[]
            {
                Primitives.Cone(shape.Radius, shape.Height, 20, MeshMaterial.Green)
            });
            Body = body;

            _radius = shape.Radius;
            _height = shape.Height;
        }

        public override void Scale()
        {
            var shape = (ConeShape)Body.CollisionShape;
            var radiusRatio = _radius / shape.Radius;
            var heightRatio = _height / shape.Height;
            Body.CollisionShape.LocalScaling *= new BulletSharp.Math.Vector3(radiusRatio, heightRatio, radiusRatio);
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
