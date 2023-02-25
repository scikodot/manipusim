using System;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic;

namespace Physics
{
    public class BoxCollider : Collider
    {
        private Vector3 _size;
        public Vector3 Size
        {
            get => _size;
            set
            {
                _size = value;
                Scale();
            }
        }

        public BoxCollider(PhysicsHandler handler, RigidBody body, RigidBodyType type) : base(handler, body, type)
        {
            var shape = body.CollisionShape as BoxShape;
            _size = shape.HalfExtentsWithMargin;
        }

        public override void Scale()
        {
            Body.CollisionShape.LocalScaling *= _size / (Body.CollisionShape as BoxShape).HalfExtentsWithMargin;
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
