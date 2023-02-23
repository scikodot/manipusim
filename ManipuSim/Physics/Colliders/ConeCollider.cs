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

        public ConeCollider(RigidBody body, RigidBodyType type) : base(body, type)
        {
            if (body.CollisionShape is not ConeShape shape)
                throw new ArgumentException($"Expected {nameof(ConeShape)} collision shape; got {body.CollisionShape.GetType().Name}.");

            _radius = shape.Radius;
            _height = shape.Height;

            // TODO: why does Primitives.Cylinder return a Mesh instead of a Model?
            Model = new Model(new Mesh[]
            {
                Primitives.Cone(_radius, _height, 20, MeshMaterial.Green)
            });
            Body = body;
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
