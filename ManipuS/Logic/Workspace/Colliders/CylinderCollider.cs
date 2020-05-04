using System;
using System.Collections.Generic;
using System.Numerics;

using BulletSharp;
using BulletSharp.Math;
using OpenTK.Graphics.OpenGL4;

using Graphics;
using Physics;

using Vector3 = System.Numerics.Vector3;

namespace Logic
{
    class CylinderCollider : Collider
    {
        private float _radius;

        public CylinderCollider(RigidBody body)
        {
            var shape = (CylinderShape)body.CollisionShape;

            if (shape == null)
                throw new ArgumentException("The body shape does not match the collider!", "body.CollisionShape");

            Model = Primitives.Cylinder(shape.Radius, shape.HalfExtentsWithMargin.Z, shape.HalfExtentsWithMargin.Z, 20, MeshMaterial.Green);
            Body = body;

            _radius = shape.Radius;
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
