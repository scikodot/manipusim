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

        private Matrix _initialState;
        public Matrix InitialState => _initialState;

        private Matrix _state;
        public Matrix State 
        {
            get => _state;
            set
            {
                _state = value;

                // propagate the provided state to all views
                Model.State = _state.ToOpenTK();
                Collider.State = _state;
            }
        }

        public float Radius => (Collider as SphereCollider).Radius;
        public bool ShowCollider { get; set; }

        public Vector3 InitialPosition
        {
            get => _initialState.Origin;
            set => _initialState.Origin = value;
        }
        public Vector3 Position => _state.Origin;

        public Vector3 InitialAxis
        {
            get => _initialState.GetRotation().Rotate(Vector3.UnitY);
            set
            {
                value.Normalize();
                var cross = Vector3.Cross(Vector3.UnitY, value);
                var dot = Vector3.Dot(Vector3.UnitY, value);
                _initialState.SetRotation(Quaternion.RotationAxis(cross, (float)Math.Acos(dot)), out _initialState);
            }
        }
        public Vector3 Axis => _state.GetRotation().Rotate(Vector3.UnitY);

        private float _coordinate;
        public float Coordinate
        {
            get => _coordinate;
            set => _coordinate = MathUtil.Clamp(value, 
                _coordinateRange.Min * MathUtil.SIMD_RADS_PER_DEG,
                _coordinateRange.Max * MathUtil.SIMD_RADS_PER_DEG);
        }

        private (float Min, float Max) _coordinateRange;
        public (float Min, float Max) CoordinateRange
        {
            get => _coordinateRange;
            set
            {
                if (value.Min > value.Max)
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
            InitialAxis = axis ?? (isEndEffector ? Vector3.UnitY : Vector3.UnitX);
            InitialPosition = position ?? Vector3.Zero;
            Coordinate = coordinate;
            CoordinateRange = (coordinateL, coordinateU);

            Model.RenderFlags = RenderFlags.Default | RenderFlags.Wireframe | RenderFlags.Lighting;
        }

        // TODO: this is individual for each joint type
        public (Vector3 Linear, Vector3 Angular) GetTwist() => (-Vector3.Cross(Axis, Position), Axis);

        public Matrix GetTransform(float coordinate)
        {
            var (linearAxis, angularAxis) = GetTwist();
            var rotation = Quaternion.RotationAxis(angularAxis, coordinate);
            var translation = (Quaternion.Identity - rotation).Rotate(Vector3.Cross(angularAxis, linearAxis))
                + angularAxis * Vector3.Dot(angularAxis, linearAxis) * coordinate;

            return Matrix.AffineTransformation(1f, rotation, translation);
        }

        public void Translate(Vector3 translation)
        {
            State *= Matrix.Translation(translation.X, translation.Y, translation.Z);

            InitialPosition += translation;
            
            // invoke the event for the containing manipulator
            TranslationChanged?.Invoke(this, new TranslationEventArgs(translation));
        }

        public void Update(ref Matrix transform, float coordinate)
        {
            //Collider.Scale();

            State = InitialState * (transform *= GetTransform(coordinate));
        }

        public void Render(ShaderProgram shader)
        {
            Model.Render(shader);

            if (ShowCollider)
                Collider.Render(shader);
        }

        public Joint Copy() => new(Model.Copy(), Collider.Copy());

        public void Dispose()
        {
            Model.Dispose();
            Collider.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
