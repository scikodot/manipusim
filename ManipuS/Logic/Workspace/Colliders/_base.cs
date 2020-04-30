using System.Numerics;

using BulletSharp;

using Graphics;
using Physics;

namespace Logic
{
    public enum ColliderShape
    {
        Box,
        Sphere
    }

    public class Collider
    {
        public Vector3[] Data;
        public Vector3 Center { get; set; }
        //public BulletSharp.Math.Vector3 Center => Body.CenterOfMassPosition;

        public Model Model;
        public RigidBody Body;

        public CollisionCallback CollisionCallback;

        protected Collider(Vector3[] data)
        {
            Data = data;
        }

        protected Collider(Model model, RigidBody body)
        {

        }

        public virtual bool Contains(Vector3 point)
        {
            return default;
        }

        public virtual Vector3 Extrude(Vector3 point)
        {
            return default;
        }

        public virtual void Render(Shader shader, ref Matrix4 state)
        {
            
        }

        public bool CollisionTest(PhysicsHandler physics, Collider other)
        {
            //physics.World.ContactPairTest(Body, other.Body, CollisionCallback);
            return CollisionCallback.CollisionTest(physics, Body, other.Body);
        }
    }
}