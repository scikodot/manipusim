using System;

using BulletSharp;

using Graphics;

using Vector3 = System.Numerics.Vector3;

namespace Logic
{
    public class BoxCollider : Collider
    {
        private Vector3 _size;

        public BoxCollider(RigidBody body)
        {
            var shape = (BoxShape)body.CollisionShape;

            if (shape == null)
                throw new ArgumentException("The body shape does not match the collider!", "body.CollisionShape");

            Model = Primitives.Cube(shape.HalfExtentsWithMargin.X, shape.HalfExtentsWithMargin.Y, shape.HalfExtentsWithMargin.Z, MeshMaterial.Green);
            Body = body;

            _size = new Vector3(shape.HalfExtentsWithMargin.X, shape.HalfExtentsWithMargin.Y, shape.HalfExtentsWithMargin.Z);
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
