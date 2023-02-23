using System;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;

using Graphics;
using BulletSharp.Math;
using Vector3 = System.Numerics.Vector3;

namespace Physics
{
    public abstract class Collider : IDisposable
    {
        private readonly PhysicsHandler _physicsHandler;
        
        public RigidBody Body { get; protected set; }
        public RigidBodyType Type { get; protected set; }
        public Model Model { get; protected set; }
        public CollisionCallback CollisionCallback { get; protected set; }

        public BroadphaseNativeType Shape => Body.CollisionShape.ShapeType;
        

        public Collider(RigidBody body, RigidBodyType type)
        {
            Body = body;
            Type = type;
            CollisionCallback = new CollisionCallback(body, null);

            UpdateModel();
        }

        public abstract bool Contains(Vector3 point);

        public abstract Vector3 Extrude(Vector3 point);

        public void Render(Shader shader)
        {
            Model.Render(shader, () =>
            {
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
                GL.DrawElements(PrimitiveType.Triangles, Model.Meshes[0].Indices.Length, DrawElementsType.UnsignedInt, 0);
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            });
        }

        public void UpdateModel()
        {
            var state = Matrix.Scaling(Body.CollisionShape.LocalScaling) * Body.MotionState.WorldTransform;
            Model.State = state.TopOpenTK();
        }

        public abstract void Scale();

        public void Reset()
        {
            //PhysicsHandler.CleanRigidBody(Body);
        }

        public bool CollisionPairTest(Collider other)
        {
            return _physicsHandler.ContactPairTest(Body, other.Body, CollisionCallback);
        }

        public bool CollisionTest()
        {
            return _physicsHandler.ContactTest(Body, CollisionCallback);
        }

        // TODO: consider making changes to mass in a separate method
        public void Convert(RigidBodyType type, float? mass = null)
        {
            // temporarily remove the body from the world
            _physicsHandler.RemoveRigidBody(Body);

            // set the given body type
            Body.SetType(type);

            // set mass properties if needed
            if (mass != null)
                Body.SetMassProps(mass.Value, Body.CollisionShape.CalculateLocalInertia(mass.Value));

            // return the body to the world
            _physicsHandler.AddRigidBody(Body);
        }

        //public void AttachGhost()
        //{
        //    // add a callback for the ghost object to detect collisions
        //    PhysicsHandler.World.Broadphase.OverlappingPairCache.SetInternalGhostPairCallback(new GhostPairCallback());

        //    // attach a ghost object to the collider
        //    Ghost = new GhostObject();
        //    Ghost.CollisionShape = Body.CollisionShape;
        //    Ghost.WorldTransform = Body.WorldTransform;
        //    Ghost.CollisionFlags |= CollisionFlags.NoContactResponse;

        //    // add the ghost object to the world
        //    PhysicsHandler.World.AddCollisionObject(Ghost, CollisionFilterGroups.SensorTrigger, CollisionFilterGroups.AllFilter);
        //}

        //public bool CollisionTestGhost()
        //{
        //    if (Ghost != null)
        //    {
        //        // if base ---> ignore

        //        // if prev/next neighbour ---> ignore

        //        // if any original manipulator collider ---> ignore

        //        Console.WriteLine($"Collisions: {Ghost.NumOverlappingObjects}");
        //        return Ghost.NumOverlappingObjects > 0;
        //    }
        //    else
        //        throw new ArgumentNullException("The ghost object is undefined!", "Ghost");
        //}

        public Collider Copy()
        {
            // body cannot be cloned directly, so the only way is to re-initialize it
            Collider collider = _physicsHandler.CreateCollider(Type, Body.CollisionShape, Body.MotionState.WorldTransform, 1f / Body.InvMass);

            // disable collider to prevent it from producing collision impulses
            collider.Body.CollisionFlags |= CollisionFlags.NoContactResponse;

            // copy callback
            collider.CollisionCallback = new CollisionCallback(collider.Body, null);

            return collider;
        }

        public void Dispose()
        {
            // clear managed resources
            Model.Dispose();
            _physicsHandler.DisposeRigidBody(Body);

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}