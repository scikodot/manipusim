using System;
using System.Collections.Generic;
using System.Linq;

using OpenTK;
using OpenTK.Input;

using MoreLinq;
using Logic;

using Matrix4 = OpenTK.Matrix4;

namespace Graphics
{
    public class TranslationalWidget : IDisposable
    {
        private class Axis : IDisposable
        {
            private Model Model { get; set; }
            private Vector3 Offset { get; set; }
            private bool FirstClick { get; set; } = true;

            public Vector3 Origin { get; private set; }
            public Vector3 Direction { get; private set; }
            public Vector3 End { get; private set; }

            public bool Active { get; private set; }

            private const float _scaleFactor = 0.2f;  // this value defines a constant size of axis on the screen
            private float _scale;

            public Axis(Vector3 origin, Vector3 direction, Vector4 color)
            {
                Origin = origin;
                Direction = direction.Normalized();

                _scale = _scaleFactor;
                End = Origin + _scale * Direction;

                Model = new Model(new MeshVertex[]
                {
                new MeshVertex { Position = Origin },
                new MeshVertex { Position = Origin + Direction }
                }, material: new MeshMaterial { Diffuse = color });
            }

            public void SetOrigin(Vector3 origin)
            {
                Translate(origin - Origin);
            }

            public void Render(Shader shader, Action render)
            {
                Model.Render(shader, render);
            }

            public void Translate(Vector3 translation)
            {
                // update edge points
                Origin += translation;
                End += translation;

                // update axis model
                ref var model = ref Model.State;
                model.M14 += translation.X;
                model.M24 += translation.Y;
                model.M34 += translation.Z;
            }

            public void Scale(Camera camera)
            {
                // obtain scale change
                _scale = _scaleFactor * (camera.Position - Origin).Length;

                // scale axis points
                End = Origin + _scale * Direction;

                // scale axis model
                ref var modelX = ref Model.State;
                modelX.M11 = modelX.M22 = modelX.M33 = _scale;
            }

            public Vector3 Poll(Camera camera, Ray ray, MouseState currState, MouseState prevState)  // TODO: optimize
            {
                if (currState.LeftButton == ButtonState.Pressed && prevState.LeftButton == ButtonState.Released)
                {
                    // button was pressed ---> start transformation
                    Active = true;
                }
                else if (currState.LeftButton == ButtonState.Released && prevState.LeftButton == ButtonState.Pressed)
                {
                    // button was released ---> stop transformation
                    Active = false;
                    FirstClick = true;
                }

                if (Active)
                {
                    // project axis onto the view plane
                    var axisView = Logic.Geometry.VectorPlaneProjection(  // TODO: refactor?
                        Direction.ToNumerics3(),
                        camera.Front.ToNumerics3()).ToOpenTK();

                    // find vector orthogonal to the projected axis
                    var axisViewOrtho = Vector3.Cross(axisView, camera.Front);

                    // construct a ray plane
                    var planeNormal = Vector3.Cross(axisViewOrtho, ray.Direction.Xyz);

                    // find the point of intersection between the ray plane and the axis
                    var intersection = Logic.Geometry.LinePlaneIntersection(  // TODO: refactor?
                        Origin.ToNumerics3(),
                        Direction.ToNumerics3(),
                        ray.StartWorld.Xyz.ToNumerics3(),
                        planeNormal.ToNumerics3()).ToOpenTK();

                    if (FirstClick)
                    {
                        // at first click memoize the offset of cursor from end point
                        Offset = intersection - End;
                        FirstClick = false;
                    }
                    else
                    {
                        // retrieve translation for the axis (in World space)
                        return intersection - Offset - End;
                    }
                }

                return default;
            }

            public bool IsActive(ref Matrix4 view, ref Matrix4 proj, out Vector3 endNDC)
            {
                // transform the axis to the NDC space
                endNDC = Project(ref view, ref proj);

                // return the distance between the projected axis tip and the cursor
                return Vector2.Distance(InputHandler.CursorPositionNDC, endNDC.Xy) < 0.1f;
            }

            public Vector3 Project(ref Matrix4 view, ref Matrix4 proj)
            {
                // transform end point to NDC
                var endProj = new Vector4(End, 1.0f) * view * proj;
                return (endProj / endProj.W).Xyz;
            }

            public void Dispose()
            {
                // clear managed resources
                Model.Dispose();

                // suppress finalization
                GC.SuppressFinalize(this);
            }
        }

        private Axis[] Axes { get; set; }
        private Axis ActiveAxis { get; set; }
        public ITranslatable Parent { get; private set; }

        public bool IsAttached => Parent != null;
        public bool IsActive => ActiveAxis != null;

        public TranslationalWidget(Vector3 origin, IEnumerable<(Vector3, Vector4)> axesDirectionsColors)
        {
            Axes = axesDirectionsColors.Select(((Vector3 dir, Vector4 col) axis) => new Axis(origin, axis.dir, axis.col)).ToArray();
        }

        public TranslationalWidget(Vector3 origin, IEnumerable<(Vector3, Vector4)> axesDirectionsColors, ITranslatable selectable)
        {
            Axes = axesDirectionsColors.Select(((Vector3 dir, Vector4 col) axis) => new Axis(origin, axis.dir, axis.col)).ToArray();

            Attach(selectable);
        }

        public void Render(Shader shader, Action render)
        {
            if (IsAttached && Parent.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                foreach (var axis in Axes)
                    axis.Render(shader, render);
        }

        public void Attach(ITranslatable selectable)
        {
            if (selectable == Parent || selectable == null)
                return;

            var selectablePosition = selectable.Collider.Body.MotionState.WorldTransform.Origin;
            foreach (var axis in Axes)
            {
                axis.SetOrigin(selectablePosition.ToOpenTK3());
            }

            Parent = selectable;
        }

        public void Detach()
        {
            Parent = null;
        }

        public void Poll(Camera camera, Ray ray, MouseState currState, MouseState prevState)
        {
            if (IsAttached)
            {
                // scale all axes so that their size on screen remains fixed
                Scale(camera);  // TODO: try to implement event-based system

                // get current view and projection matrices
                ref var view = ref camera.ViewMatrix;
                ref var proj = ref camera.ProjectionMatrix;  // TODO: make matrices ref properties

                if (ActiveAxis == null || (ActiveAxis != null && !ActiveAxis.Active))
                {
                    // get active priority axis
                    ActiveAxis = GetActiveAxis(ref view, ref proj);
                }

                if (ActiveAxis != null)
                {
                    // poll axis for interaction
                    var translation = ActiveAxis.Poll(camera, ray, currState, prevState);

                    // translate the parent object and the widget with the acquired translation
                    Translate(translation);
                }

            }
        }

        private void Translate(Vector3 translation)
        {
            // translate the parent object
            Parent.Translate(translation.ToNumerics3());

            // translate the widget axes
            foreach (var axis in Axes)
                axis.Translate(translation);
        }

        private void Scale(Camera camera)
        {
            foreach (var axis in Axes)
                axis.Scale(camera);
        }

        private Axis GetActiveAxis(ref Matrix4 view, ref Matrix4 proj)
        {
            var axesActive = new List<(Axis, Vector3)>();
            foreach (var axis in Axes)
            {
                if (axis.IsActive(ref view, ref proj, out Vector3 endNDC))
                    axesActive.Add((axis, endNDC));
            }

            return axesActive.Count == 0 ? null : axesActive.MinBy(x => Math.Abs(x.Item2.Z)).First().Item1;
        }

        public void Dispose()
        {
            // clear managed resources
            foreach (var axis in Axes)
            {
                axis.Dispose();
            }

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}