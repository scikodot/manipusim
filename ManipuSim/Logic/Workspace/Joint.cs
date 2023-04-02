using System;
using System.Threading.Tasks;
using BulletSharp;
using BulletSharp.Math;
using Graphics;
using Physics;

namespace Logic
{
    public enum JointType
    {
        Prismatic,  // Translation
        Revolute,  // Rotation
        Cylindrical,  // Translation & rotation
        Spherical,  // Allows three degrees of rotational freedom about the center of the joint. Also known as a ball-and-socket joint
        Planar  // Allows relative translation on a plane and relative rotation about an axis perpendicular to the plane
    }

    public class TranslationEventArgs : EventArgs
    {
        public Vector3 Translation { get; }

        public TranslationEventArgs(Vector3 translation)
        {
            Translation = translation;
        }
    }

    public class Joint : IDisposable, ISelectable, ITranslatable  // TODO: consider using abstract class Selectable instead of interface
    {
        private static Model _defaultModel;

        // TODO: consider creating a separate EndEffector/Tool class
        private static Model _defaultGripperModel;

        public Model Model { get; private set; }
        public Collider Collider { get; private set; }

        public Matrix State => Collider.State;

        public float Radius => (Collider as SphereCollider).Radius;
        public bool ShowCollider { get; set; }

        public Vector3 InitialPosition { get; set; }
        public Vector3 Position { get; set; }
        public Vector3 InitialAxis { get; set; }
        public Vector3 Axis { get; set; }
        public float InitialCoordinate { get; }
        public float Coordinate { get; set; }

        private (float, float) _coordinateRange;
        public (float l, float u) CoordinateRange
        {
            get => _coordinateRange;
            set
            {
                if (value.l > value.u)
                    throw new ArgumentException("Invalid coordinate range.");

                _coordinateRange = value;
            }
        }
        public bool Active { get; set; }

        public event EventHandler<TranslationEventArgs> TranslationChanged;

        public static void LoadDefaultModel(string jointPath, string gripperPath)
        {
            Dispatcher.ActiveTasks.Add(Task.Run(() =>
            {
                _defaultModel = new Model(jointPath);
                _defaultGripperModel = new Model(gripperPath);
            }));
        }

        public Joint(Model model = null, Collider collider = null, Vector3? axis = null, Vector3? position = null, 
            float coordinate = 0, float coordinateL = -180, float coordinateU = 180, bool isEndEffector = false)
        {
            Model = model ?? (isEndEffector ? _defaultGripperModel.Copy() : _defaultModel.Copy());
            Collider = collider ?? (isEndEffector ? 
                Collider.Create(new SphereShape(0.1f)) : 
                Collider.Create(new SphereShape(0.2f)));
            Collider.Body.UserObject = this;
            Axis = InitialAxis = axis ?? (isEndEffector ? Vector3.UnitY : Vector3.UnitX);
            Position = InitialPosition = position ?? Vector3.Zero;
            Coordinate = InitialCoordinate = coordinate;
            CoordinateRange = (coordinateL, coordinateU);

            Model.RenderFlags = RenderFlags.Default | RenderFlags.Wireframe | RenderFlags.Lighting;
        }

        public void Translate(Vector3 translation)
        {
            State *= Matrix.Translation(translation.X, translation.Y, translation.Z);

            InitialPosition += translation;
            
            // invoke the event for the containing manipulator
            TranslationChanged?.Invoke(this, new TranslationEventArgs(translation));
        }

        public Joint Copy() => new(Model.Copy(), Collider.Copy());

        public void Render(ShaderProgram shader)
        {
            Model.Render(shader);

            if (ShowCollider)
                Collider.Render(shader);
        }

        public void Update(InteractionMode mode)
        {
            //Collider.Scale();
            var state = State.ToOpenTK();
            Model.Update(state);
            Collider.Model.Update(state);
        }

        public void UpdateState(in ImpDualQuat state)
        {
            
        }

        public void Dispose()
        {
            Model.Dispose();
            Collider.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
