﻿using System;
using System.Data;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic;

namespace Physics
{
    class SphereCollider : Collider
    {
        private float _radius;
        public ref float Radius => ref _radius;

        public SphereCollider(PhysicsHandler handler, RigidBody body, RigidBodyType type) : base(handler, body, type)
        {
            var shape = body.CollisionShape as SphereShape;
            _radius = shape.Radius;
        }

        public override void Scale()
        {
            Body.CollisionShape.LocalScaling *= (Radius / ((SphereShape)Body.CollisionShape).Radius) * BulletSharp.Math.Vector3.One;
        }

        public override bool Contains(Vector3 point)
        {
            return Vector3.Distance(Body.CenterOfMassPosition, point) <= _radius;
        }

        public override Vector3 Extrude(Vector3 point)
        {
            var center = Body.CenterOfMassPosition;
            Vector3 vec = point - new Vector3(center.X, center.Y, center.Z);
            return Vector3.Normalize(vec) * _radius - vec;
        }
    }
}
