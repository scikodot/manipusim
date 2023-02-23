using System;

using BulletSharp;

using Graphics;
using Logic;

using Vector3 = System.Numerics.Vector3;

namespace Physics
{
    public class BoxCollider : Collider
    {
        private Vector3 _size;
        public ref Vector3 Size => ref _size;

        public BoxCollider(RigidBody body, RigidBodyType type) : base(body, type)
        {
            if (body.CollisionShape is not BoxShape shape)
                throw new ArgumentException($"Expected {nameof(BoxShape)} collision shape; got {body.CollisionShape.GetType().Name}.");

            _size = shape.HalfExtentsWithMargin.ToNumerics3();

            Model = Primitives.Cube(_size.X, _size.Y, _size.Z, MeshMaterial.Green);
            Body = body;
        }

        public override void Scale()
        {
            Body.CollisionShape.LocalScaling *= new BulletSharp.Math.Vector3(_size.X, _size.Y, _size.Z) / ((BoxShape)Body.CollisionShape).HalfExtentsWithMargin;
        }

        public override bool Contains(Vector3 point)
        {
            var center = Body.CenterOfMassPosition;
            return point.X >= center.X - _size.X && point.X <= center.X + _size.X &&
                   point.Y >= center.Y - _size.Y && point.Y <= center.Y + _size.Y &&
                   point.Z >= center.Z - _size.Z && point.Z <= center.Z + _size.Z;
        }

        public override Vector3 Extrude(Vector3 point)  // TODO: this method behaves weirdly; repair
        {
            var center = Body.CenterOfMassPosition;
            Vector3 vec = point - new Vector3(center.X, center.Y, center.Z);
            Vector3 diff = _size - vec;
            float ratio;
            if (diff.X > diff.Y)
            {
                if (diff.X > diff.Z)
                {
                    if (diff.Y > diff.Z)
                        ratio = diff.Z / vec.Z;
                    else
                        ratio = diff.Y / vec.Y;
                }
                else
                    ratio = diff.Y / vec.Y;
            }
            else
            {
                if (diff.Y > diff.Z)
                {
                    if (diff.X > diff.Z)
                        ratio = diff.Z / vec.Z;
                    else
                        ratio = diff.X / vec.X;
                }
                else
                    ratio = diff.X / vec.X;
            }

            return vec * ratio;
        }
    }
}
