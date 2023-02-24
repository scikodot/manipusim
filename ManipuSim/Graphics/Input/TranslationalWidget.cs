using System;
using System.Collections.Generic;
using System.Linq;

using OpenTK.Mathematics;
using OpenTK.Windowing.GraphicsLibraryFramework;

using Logic;

using Matrix4 = OpenTK.Mathematics.Matrix4;
using Assimp;

namespace Graphics
{
    public class TranslationalWidget : IDisposable
    {
        private class Axis : IDisposable
        {
            private const float _scaleFactor = 0.15f;  // determines a constant axis size on the screen

            private readonly Model _model;
            private Vector3 _offset;
            private float _scale;

            public Vector3 Origin { get; private set; }
            public Vector3 Direction { get; private set; }
            public Vector3 End { get; private set; }

            public Axis(Vector3 origin, Vector3 direction, Vector4 color)
            {
                Origin = origin;
                Direction = direction.Normalized();

                _scale = _scaleFactor;
                End = Origin + _scale * Direction;

                _model = CreateModel(Origin, Direction, color);
            }

            public Model CreateModel(Vector3 origin, Vector3 direction, Vector4 color)
            {
                // TODO: create Align method for matrices, the same as for quaternions
                Matrix4 align = Matrix4.CreateTranslation(origin);
                if (direction != Vector3.UnitY)
                    align *= Matrix4.CreateFromAxisAngle(Vector3.Cross(Vector3.UnitY, direction), (float)Math.Acos(Vector3.Dot(Vector3.UnitY, direction)));

                return new Model(new Mesh[]
                {
                    Primitives.Cylinder(0.01f, 0, 1f, 20, new MeshMaterial
                    {
                        Diffuse = color
                    }),
                    Primitives.Cone(0.05f, 0.2f, 20, new MeshMaterial
                    {
                        Diffuse = color
                    }, Vector3.UnitY)
                }, align * Matrix4.CreateScale(_scale));
            }

            public void SetOrigin(Vector3 origin)
            {
                Translate(origin - Origin);
            }

            public void Render(Shader shader, Action action)
            {
                _model.Render(shader, action);
            }

            public void Translate(Vector3 translation)
            {
                // update edge points
                Origin += translation;
                End += translation;

                // update axis model
                ref var model = ref _model.State;
                model.M41 += translation.X;
                model.M42 += translation.Y;
                model.M43 += translation.Z;
            }

            public void Scale(Vector3 cameraPosition)
            {
                // get the new scale
                var scale = _scaleFactor * (cameraPosition - Origin).Length;

                // hasn't changed -> no need to update
                if (scale == _scale)
                    return;

                // scale axis points
                End = Origin + scale * Direction;

                // scale axis model
                ref var modelX = ref _model.State;
                modelX = Matrix4.CreateScale(scale / _scale) * modelX;

                // save the new scale
                _scale = scale;
            }

            public void Setup(Ray ray)
            {
                // memoize the offset of the cursor from the axis tip
                _offset = GetAxisCursorIntersection(ray) - End;
            }

            public Vector3 Poll(Ray ray)
            {
                // get world space translation for the axis
                return GetAxisCursorIntersection(ray) - End - _offset;
            }

            private Vector3 GetAxisCursorIntersection(Ray ray)
            {
                // TODO: optimize
                // TODO: the intersection check can probably be simplified:
                // 1. Find the axis projection onto the view plane
                // 2. Find the ray intersection point with the view plane (it must be the ray's start point)
                // 3. Check whether these two overlap; this can be done by projecting the point onto the axis projection line
                //    and then checking if the point projection is between the start and end of the axis projection

                // project axis onto the view plane
                var axisView = Geometry.VectorPlaneProjection(  // TODO: refactor?
                    Direction.ToNumerics3(),
                    ray.CameraFront.ToNumerics3()).ToOpenTK();

                // find vector orthogonal to the projected axis
                var axisViewOrtho = Vector3.Cross(axisView, ray.CameraFront);

                // construct a ray plane
                var planeNormal = Vector3.Cross(axisViewOrtho, ray.Direction.Xyz);

                // find the point of intersection between the ray plane and the axis
                var intersection = Geometry.LinePlaneIntersection(  // TODO: refactor?
                    Origin.ToNumerics3(),
                    Direction.ToNumerics3(),
                    ray.StartWorld.Xyz.ToNumerics3(),
                    planeNormal.ToNumerics3()).ToOpenTK();

                return intersection;
            }

            /*public bool IsInteracted(Ray ray)
            {
                // treat an axis tip as a cylinder and find its intersection with the ray
                var distance = Geometry.LinesDistance(
                    Origin.ToNumerics3(),
                    Direction.ToNumerics3(),
                    ray.StartWorld.ToNumerics3(),
                    ray.Direction.ToNumerics3());

                return distance <= 0.05f;
            }*/

            public bool IsPrioritized(Ray ray, out float priority)
            {
                // TODO: the best way is to get a line (= ray) to cone (= arrow tip) intersection,
                // but it requires an additional method LineConeIntersection() to be implemented

                // find the distance between the axis tip (sphere approx) and a ray
                var origin = Origin + Direction * _scale;
                var distance = Geometry.PointLineDistance(
                    origin.ToNumerics3(), 
                    ray.StartWorld.ToNumerics3(), 
                    ray.Direction.ToNumerics3());

                // priority is the distance between the axis tip and a ray's origin, 
                // but it is only defined if the ray comes close enough to the tip
                bool prioritized = distance <= 0.1f * _scale;
                priority = prioritized ? (origin - ray.StartWorld.Xyz).Length : float.MaxValue;

                return prioritized;
            }

            public (Vector3, Vector3) Project(ref Matrix4 view, ref Matrix4 proj)
            {
                // transform end point to NDC
                var viewProj = view * proj;
                var originProj = new Vector4(Origin, 1.0f) * viewProj;
                var endProj = new Vector4(End, 1.0f) * viewProj;
                return ((endProj / endProj.W).Xyz, (originProj / originProj.W).Xyz);
            }

            public void Dispose()
            {
                // clear managed resources
                _model.Dispose();

                // suppress finalization
                GC.SuppressFinalize(this);
            }
        }

        private readonly Axis _axisX;
        private readonly Axis _axisY;
        private readonly Axis _axisZ;
        private Axis _activeAxis;
        private ITranslatable _parent;

        public bool IsAttached => _parent != null;
        public bool IsActive => _activeAxis != null;

        public TranslationalWidget(Vector3 origin)
        {
            // standard right-handed system with RGB color scheme
            _axisX = new Axis(origin, new Vector3(1, 0, 0), new Vector4(1, 0, 0, 1));
            _axisY = new Axis(origin, new Vector3(0, 1, 0), new Vector4(0, 1, 0, 1));
            _axisZ = new Axis(origin, new Vector3(0, 0, 1), new Vector4(0, 0, 1, 1));
        }

        public void Render(Shader shader, Action action)
        {
            if (IsAttached && _parent.Model.RenderFlags.HasFlag(RenderFlags.Selected))
            {
                _axisX.Render(shader, action);
                _axisY.Render(shader, action);
                _axisZ.Render(shader, action);
            }
        }

        public void OnSelectedObjectChanged(ObjectSelectEventArgs e)
        {
            _parent = e.Object as ITranslatable;

            // attach the widget only if the selected object is translatable
            if (_parent != null)
            {
                var origin = _parent.Collider.Body.MotionState.WorldTransform.Origin.ToOpenTK3();
                _axisX.SetOrigin(origin);
                _axisY.SetOrigin(origin);
                _axisZ.SetOrigin(origin);
            }
        }

        public void TryActivate(Ray ray)
        {
            if (!IsAttached)
                throw new ArgumentException("Attempt to activate an unattached widget.");

            // get active priority axis
            _activeAxis = GetActiveAxis(ray);
            _activeAxis?.Setup(ray);
        }

        public void Operate(Ray ray)
        {
            if (!IsActive)
                throw new ArgumentException("Attempt to operate a widget while no axis is active.");

            // poll the active axis for translation
            Translate(_activeAxis.Poll(ray));
        }

        public void Deactivate()
        {
            _activeAxis = null;
        }

        private void Translate(Vector3 translation)
        {
            // translate the parent object
            _parent.Translate(translation.ToNumerics3());

            // translate the widget axes
            _axisX.Translate(translation);
            _axisY.Translate(translation);
            _axisZ.Translate(translation);
        }

        public void Scale(Vector3 cameraPosition)
        {
            _axisX.Scale(cameraPosition);
            _axisY.Scale(cameraPosition);
            _axisZ.Scale(cameraPosition);
        }

        private Axis GetActiveAxis(Ray ray)
        {
            //var axes = new List<(Axis, Vector3)>();
            //bool interactedX = _axisX.IsInteracted(ray);
            //bool interactedY = _axisY.IsInteracted(ray);
            //bool interactedZ = _axisZ.IsInteracted(ray);

            // active axis is the one with the lowest priority,
            // i.e. with the smallest distance from its tip to the view plane
            var priorities = new Dictionary<Axis, float>();
            if (_axisX.IsPrioritized(ray, out float priorityX)) priorities.Add(_axisX, priorityX);
            if (_axisY.IsPrioritized(ray, out float priorityY)) priorities.Add(_axisY, priorityY);
            if (_axisZ.IsPrioritized(ray, out float priorityZ)) priorities.Add(_axisZ, priorityZ);

            return priorities.Count == 0 ? null : priorities.MinBy(kvp => kvp.Value).Key;
        }

        public void Dispose()
        {
            // clear managed resources
            _axisX.Dispose();
            _axisY.Dispose();
            _axisZ.Dispose();

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}