using BulletSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace Physics
{
    public class CollisionCallback : ContactResultCallback
    {
        private RigidBody _monitoredBody;
        private object _context; // External information for contact processing

        private bool _contactDetected;
        public bool ContactDetected 
        { 
            get
            {
                if (_contactDetected)
                {
                    _contactDetected = false;
                    return true;
                }
                else
                    return false;
            }
            set => _contactDetected = value;
        }

        // Constructor, pass whatever context you want to have available when processing contacts.
        // You may also want to set CollisionFilterGroups and CollisionFilterMask
        //  (supplied by the superclass) for NeedsCollision().
        public CollisionCallback(RigidBody monitoredBody, object context /*, ... */)
        {
            _monitoredBody = monitoredBody;
            _context = context;
        }

        public CollisionCallback(CollisionObject collisionObject, object context /*, ... */)
        {
            _monitoredBody = collisionObject as RigidBody;
            _context = context;
        }

        // If you don't want to consider collisions where the bodies are joined by a constraint, override NeedsCollision:
        // However, if you use a CollisionObject for #body instead of a RigidBody,
        //  then this is unnecessary — CheckCollideWithOverride isn't available.
        public override bool NeedsCollision(BroadphaseProxy proxy)
        {
            // superclass will check CollisionFilterGroup and CollisionFilterMask
            if (base.NeedsCollision(proxy))
            {
                // if passed filters, may also want to avoid contacts between constraints
                //return body.CheckCollideWithOverride(proxy.ClientObject as CollisionObject);
            }

            return false;
        }

        // Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
        public override float AddSingleResult(ManifoldPoint contact,
            CollisionObjectWrapper colObj0, int partId0, int index0,
            CollisionObjectWrapper colObj1, int partId1, int index1)
        {
            Vector3 collisionPoint; // relative to body
            if (colObj0.CollisionObject == _monitoredBody)
            {
                var vec = contact.LocalPointA;
                collisionPoint = new Vector3(vec.X, vec.Y, vec.Z);
            }
            else
            {
                System.Diagnostics.Debug.Assert(colObj1.CollisionObject == _monitoredBody);
                var vec = contact.LocalPointA;
                collisionPoint = new Vector3(vec.X, vec.Y, vec.Z);
            }

            // on collision test, Bullet performs broadphase collision test with bounding spheres;
            // because of that, obtained contacts in general do not represent actual contacts between bodies;
            // hence, we have to check if the distance between those two points is small enough ...
            if (contact.Distance < 0.01f)
            {
                // ... and if it is, notify about a collision
                ContactDetected = true;
            }

            // do stuff with the collision point
            return 0; // not actually sure if return value is used for anything...?
        }
    }
}
