using System;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic;
using BulletSharp.SoftBody;

namespace Physics
{
    public abstract class Collider : IDisposable
    {
        private readonly MeshMaterial _defaultMaterial = new() { ColorDiffuse = OpenTK.Mathematics.Color4.Lime };

        private PhysicsHandler _physicsHandler;

        public RigidBody Body { get; protected set; }
        public RigidBodyType Type { get; protected set; }
        public Model Model { get; protected set; }
        public CollisionCallback CollisionCallback { get; protected set; }

        public BroadphaseNativeType Shape => Body.CollisionShape.ShapeType;
        public Matrix State
        {
            get
            {
                var state = Body.MotionState.WorldTransform;

                // apply shape's local scaling before passing the state
                var scale = Body.CollisionShape.LocalScaling;
                state.Row1 *= scale.X;
                state.Row2 *= scale.Y;
                state.Row3 *= scale.Z;

                return state;
            }
        }

        protected Vector3 _size;
        public Vector3 Size
        {
            get => _size;
            set => Scale(value / _size);
        }

        protected Vector3 _orientation;
        public Vector3 Orientation
        {
            get => _orientation;
            set => Rotate(value - _orientation);
        }

        protected Vector3 _position;
        public Vector3 Position
        {
            get => _position;
            set => Translate(value - _position);
        }

        protected Collider(RigidBody body, RigidBodyType type)
        {
            // set default orientation and position provided for the body;
            // default size is set separately for each shape
            body.WorldTransform.Decompose(out _, out var orientation, out var position);
            _orientation = orientation.ToEuler();
            _position = position;

            Body = body;
            Type = type;
            Model = new Model(new Mesh[]
            {
                Primitives.FromCollisionShape(body.CollisionShape, _defaultMaterial)
            });
            CollisionCallback = new CollisionCallback(body, null);

            Model.Update(State.ToOpenTK());
        }

        public static Collider Create(CollisionShape shape, RigidBodyType type = RigidBodyType.Kinematic, 
            Matrix transform = default, float mass = 0)
        {
            var localInertia = mass == 0 ? Vector3.Zero : shape.CalculateLocalInertia(mass);
            var motionState = new DefaultMotionState(transform == default ? Matrix.Identity : transform);
            using var rbInfo = new RigidBodyConstructionInfo(mass, motionState, shape, localInertia);
            var body = new RigidBody(rbInfo);

            // set the provided type and disable deactivation,
            // because objects are created in design mode
            body.SetType(type);
            body.ForceActivationState(ActivationState.DisableDeactivation);

            return body.CollisionShape.ShapeType switch
            {
                BroadphaseNativeType.BoxShape => new BoxCollider(body, type),
                BroadphaseNativeType.SphereShape => new SphereCollider(body, type),
                BroadphaseNativeType.CylinderShape => new CylinderCollider(body, type),
                BroadphaseNativeType.ConeShape => new ConeCollider(body, type),
                _ => throw new ArgumentException("Unknown collision shape.")
            };
        }

        public void Attach(PhysicsHandler physicsHandler)
        {
            (_physicsHandler = physicsHandler).AddRigidBody(Body);
        }

        // TODO: check usage
        public abstract bool Contains(Vector3 point);

        // TODO: check usage
        public abstract Vector3 Extrude(Vector3 point);

        public void Scale(Vector3 scaling)
        {
            if (scaling == Vector3.One)
                return;

            _size *= scaling;

            Body.CollisionShape.LocalScaling *= scaling;
        }

        public void Rotate(Vector3 rotation)
        {
            if (!Body.CollisionFlags.HasFlag(CollisionFlags.KinematicObject))
                throw new InvalidOperationException("Attempt to rotate a non-kinematic object.");

            if (rotation == Vector3.Zero)
                return;

            _orientation += rotation;

            var state = GetRotationMatrix();
            state.Origin = _position;
            Body.MotionState.WorldTransform = state;
        }

        private Matrix GetRotationMatrix()
        {
            // TODO: Y and Z axes are swapped in graphics and we account for it, but is it ok?
            var (pitch, yaw, roll) = _orientation * MathUtil.SIMD_RADS_PER_DEG;
            return Matrix.RotationYawPitchRoll(yaw, pitch, roll);
        }

        public void Translate(Vector3 translation)
        {
            if (!Body.CollisionFlags.HasFlag(CollisionFlags.KinematicObject))
                throw new InvalidOperationException("Attempt to translate a non-kinematic object.");

            if (translation == Vector3.Zero)
                return;

            _position += translation;

            var state = Body.MotionState.WorldTransform;
            state.Origin = _position;
            Body.MotionState.WorldTransform = state;
        }

        public void Render(ShaderProgram shader)
        {
            // TODO: Collider must be rendered either with
            // GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
            // or with the alpha value less than that of the object it is attached to
            Model.Render(shader);
        }

        public void Reset()
        {
            _physicsHandler?.CleanRigidBody(Body);

            var state = GetRotationMatrix();
            state.Origin = _position;
            Body.MotionState.WorldTransform = state;
        }

        public bool CollisionPairTest(Collider other)
        {
            if (_physicsHandler == null)
                throw new InvalidOperationException("Cannot collision pair test an unattached collider.");

            return _physicsHandler.ContactPairTest(Body, other.Body, CollisionCallback);
        }

        public bool CollisionTest()
        {
            if (_physicsHandler == null)
                throw new InvalidOperationException("Cannot collision test an unattached collider.");

            return _physicsHandler.ContactTest(Body, CollisionCallback);
        }

        // TODO: consider making changes to mass in a separate method
        public void Convert(RigidBodyType type, float? mass = null)
        {
            // temporarily remove the body from the world
            _physicsHandler?.RemoveRigidBody(Body);

            // set the given body type
            Body.SetType(type);

            // set mass properties if needed;
            // setting those for a static object would silently make it dynamic
            if (type != RigidBodyType.Static && mass != null)
                Body.SetMassProps(mass.Value, Body.CollisionShape.CalculateLocalInertia(mass.Value));

            // return the body to the world
            _physicsHandler?.AddRigidBody(Body);
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
            if (_physicsHandler == null)
                throw new InvalidOperationException("Cannot copy an unattached collider.");

            // body cannot be cloned directly, so the only way is to re-initialize it
            var collider = Create(Body.CollisionShape, Type, Body.MotionState.WorldTransform, 1f / Body.InvMass);

            // add the newly created body to the world
            _physicsHandler.AddRigidBody(collider.Body);

            // disable collider to prevent it from producing collision impulses
            collider.Body.CollisionFlags |= CollisionFlags.NoContactResponse;

            // copy callback
            collider.CollisionCallback = new CollisionCallback(collider.Body, null);

            return collider;
        }

        public void Dispose()
        {
            Model.Dispose();

            // remove the body from the world
            _physicsHandler?.RemoveRigidBody(Body);  // [obsolete] TODO: can crash, perhaps due to a lack of thread sync

            // then dipose of the body
            Body.MotionState.Dispose();
            Body.CollisionShape.Dispose();
            Body.Dispose();

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}