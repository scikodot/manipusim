using System;
using System.Collections.Generic;
using System.Linq;

using OpenTK;
using OpenTK.Input;

using MoreLinq;

namespace Graphics
{
    public class AxesWidget
    {
        public Axis[] Axes { get; private set; }
        public Axis ActiveAxis { get; private set; }
        public IRenderable Parent { get; private set; }

        public AxesWidget()
        {
            Axes = new Axis[3]
            {
                new Axis(Vector4.UnitW, new Vector4(0.3f, 0, 0, 1), new Vector4(1, 0, 0, 1)),
                new Axis(Vector4.UnitW, new Vector4(0, 0.3f, 0, 1), new Vector4(0, 1, 0, 1)),
                new Axis(Vector4.UnitW, new Vector4(0, 0, 0.3f, 1), new Vector4(0, 0, 1, 1))
            };
        }

        public void Render(Action render)
        {
            foreach (var axis in Axes)
                axis.Render(render);
        }

        public void Attach(IRenderable renderable)
        {
            Parent = renderable;
        }

        public void Poll(Camera camera, Vector2 cursorPos, MouseState mouseState)
        {
            // scale all axes so that their size on screen remains fixed
            Scale(camera);  // TODO: try to implement event-based system

            // get current view and projection matrices
            ref var view = ref camera.ViewMatrix;
            ref var proj = ref camera.ProjectionMatrix;  // TODO: make matrices ref properties

            if (ActiveAxis == null || (ActiveAxis != null && !ActiveAxis.Active))
            {
                // get active priority axis
                ActiveAxis = GetActiveAxis(ref view, ref proj, cursorPos);
            }

            if (ActiveAxis != null)
            {
                // poll axis for interaction
                var translation = ActiveAxis.Poll(camera, ref view, ref proj, mouseState, cursorPos);

                // translate the parent object and the widget with the acquired translation
                Translate(translation);
            }
        }

        private void Translate(Vector3 translation)
        {
            // translate the parent object
            ref var parentState = ref Parent.State;
            parentState.M14 += translation.X;
            parentState.M24 += translation.Y;
            parentState.M34 += translation.Z;

            // translate the widget axes
            foreach (var axis in Axes)
                axis.Translate(translation);
        }

        private void Scale(Camera camera)
        {
            foreach (var axis in Axes)
                axis.Scale(camera);
        }

        private Axis GetActiveAxis(ref Matrix4 view, ref Matrix4 proj, Vector2 cursorPos)
        {
            var axesActive = new List<(Axis, Vector3)>();
            foreach (var axis in Axes)
            {
                if (axis.IsActive(ref view, ref proj, cursorPos, out Vector3 endNDC))
                    axesActive.Add((axis, endNDC));
            }

            return axesActive.Count == 0 ? null : axesActive.MinBy(x => Math.Abs(x.Item2.Z)).First().Item1;
        }

        public static (Vector4, Vector4) Raycast(Vector2 ndc, ref Matrix4 view, ref Matrix4 proj)  // TODO: optimize, move somewhere else
        {
            var pStart = new Vector4(ndc.X, ndc.Y, -1, 1);
            var pEnd = new Vector4(ndc.X, ndc.Y, 0, 1);

            var pvInv = Matrix4.Invert(view * proj);

            var rayStartWorld = pStart * pvInv;
            rayStartWorld /= rayStartWorld.W;

            var rayEndWorld = pEnd * pvInv;
            rayEndWorld /= rayEndWorld.W;

            var rayDir = Vector4.Normalize(rayEndWorld - rayStartWorld);

            return (rayStartWorld, rayDir);
        }
    }

    public class Axis
    {
        private PlainModel Model { get; set; }
        private Vector3 Offset { get; set; }
        private bool FirstClick { get; set; } = true;

        public Vector4 Start { get; private set; }
        public Vector4 End { get; private set; }
        public Vector3 Direction => (End - Start).Xyz.Normalized();
        public bool Active { get; private set; }

        private float _scaleFactor = 1;
        public float ScaleFactor
        {
            get => _scaleFactor;
            private set
            {
                // determine change in scale
                var ratio = value / _scaleFactor;  // TODO: check for zero, i.e. when camera is aligned with the widget

                // apply that change
                End = ratio * (End - Start) + Start;

                // and store the new scale
                _scaleFactor = value;
            }
        }

        public Axis(Vector4 start, Vector4 end, Vector4 color)
        {
            Start = start;
            End = end;

            Model = new PlainModel(Window.lineShader, new float[]
            {
                start.X, start.Y, start.Z,   color.X, color.Y, color.Z, color.W,
                end.X, end.Y, end.Z,    color.X, color.Y, color.Z, color.W
            });
        }

        public void Render(Action render)
        {
            Model.Render(render);
        }

        internal void Translate(Vector3 translation)
        {
            // update edge points
            Start += new Vector4(translation);
            End += new Vector4(translation);

            // update axis model
            ref var model = ref Model.State;
            model.M14 += translation.X;
            model.M24 += translation.Y;
            model.M34 += translation.Z;
        }

        internal void Scale(Camera camera)
        {
            // update scale factor
            ScaleFactor = (camera.Position - Start.Xyz).Length;

            // update axis model
            ref var modelX = ref Model.State;
            modelX.M11 = modelX.M22 = modelX.M33 = ScaleFactor;
        }

        internal Vector3 Poll(Camera camera, ref Matrix4 view, ref Matrix4 proj, MouseState stateCurr, Vector2 cursorPos)  // TODO: optimize, remove state
        {
            if (stateCurr.RightButton == ButtonState.Pressed)
            {
                // button was pressed ---> start transformation
                Active = true;

                // cast a ray from the current cursor position
                (var rayOrigin, var rayDirection) = AxesWidget.Raycast(cursorPos, ref view, ref proj);

                // project axis onto the view plane
                var axisView = VectorPlaneProjection(Direction, camera.Front);

                // find vector orthogonal to the projected axis
                var axisViewOrtho = Vector3.Cross(axisView, camera.Front);

                // construct a ray plane
                var planeNormal = Vector3.Cross(axisViewOrtho, rayDirection.Xyz);

                // find the point of intersection between the ray plane and the axis
                var intersection = LinePlaneIntersection(Start.Xyz, Direction, rayOrigin.Xyz, planeNormal);

                if (FirstClick)
                {
                    // at first click memoize the offset of cursor from end point
                    Offset = intersection - End.Xyz;
                    FirstClick = false;
                }
                else
                {
                    // retrieve translation for the axis (in World space)
                    return intersection - Offset - End.Xyz;
                }
            }
            else
            {
                // button was released ---> stop transformation
                Active = false;
                FirstClick = true;
            }

            return default;
        }

        internal bool IsActive(ref Matrix4 view, ref Matrix4 proj, Vector2 cursorPos, out Vector3 endNDC)
        {
            // transform the axis to the NDC space
            endNDC = Project(ref view, ref proj);

            // return the distance between the projected axis tip and the cursor
            return Vector2.Distance(cursorPos, endNDC.Xy) < 0.1f;
        }

        internal Vector3 Project(ref Matrix4 view, ref Matrix4 proj)
        {
            // transform end point to NDC
            var endProj = End * view * proj;
            return (endProj / endProj.W).Xyz;
        }

        public Vector3 LinePlaneIntersection(Vector3 pLine, Vector3 nLine, Vector3 pPlane, Vector3 nPlane)  // TODO: move to library
        {
            var nom = Vector3.Dot(pPlane - pLine, nPlane);
            var denom = Vector3.Dot(nLine, nPlane);

            if (denom != 0)
            {
                // a unique point of intersection
                return pLine + nLine * nom / denom;
            }
            else
            {
                if (nom == 0)
                {
                    // the ray is on the plane, i.e. infinite amount of intersections
                    return default;
                }
                else
                {
                    // the ray is parallel to the plane, i.e. no intersections
                    return default;
                }
            }
        }

        public Vector3 VectorPlaneProjection(Vector3 v, Vector3 n)  // TODO: move to library
        {
            // get the component of v that is perpendicular to plane's normal, i.e. lies in that plane
            return VectorOrtho(v, n);
        }

        //public Vector3 LinesIntersection(Vector3 p1, Vector3 a1, Vector3 p2, Vector3 a2)  // TODO: move to library, try to optimize
        //{
        //    var p = p1 - p2;
        //    var f1 = Vector3.Dot(a1, p);
        //    var f2 = Vector3.Dot(a1, a1);
        //    var f3 = Vector3.Dot(a1, a2);
        //    var f4 = Vector3.Dot(a2, p);
        //    var f5 = Vector3.Dot(a2, a2);

        //    var denom = f3 * f3 - f2 * f5;
        //    if (denom != 0)
        //    {
        //        // the lines are not parallel ---> use the general formula
        //        var t2 = (f1 * f3 - f2 * f4) / denom;

        //        return p2 + t2 * a2;
        //    }
        //    else
        //    {
        //        // ???
        //        return default;
        //    }
        //}

        public float PointLineDistance(Vector3 p, Vector3 pLine, Vector3 nLine)  // TODO: move to library
        {
            return PointLineVector(p, pLine, nLine).Length;
        }

        public float LinesDistance(Vector3 p1, Vector3 n1, Vector3 p2, Vector3 n2)  // TODO: move to library
        {
            return LinesVector(p1, n1, p2, n2).Length;
        }

        private Vector3 LinesVector(Vector3 p1, Vector3 n1, Vector3 p2, Vector3 n2)  // TODO: move to library
        {
            var p = p1 - p2;
            var f1 = Vector3.Dot(n1, p);
            var f2 = Vector3.Dot(n1, n1);
            var f3 = Vector3.Dot(n1, n2);
            var f4 = Vector3.Dot(n2, p);
            var f5 = Vector3.Dot(n2, n2);

            var denom = f3 * f3 - f2 * f5;
            if (denom != 0)
            {
                // the lines are not parallel ---> use the general formula
                var nom1 = f1 * f5 - f3 * f4;
                var nom2 = f1 * f3 - f2 * f4;

                return p + (n1 * nom1 - n2 * nom2) / denom;
            }
            else
            {
                // the lines are parallel ---> use formula for a vector between a point and a line
                return PointLineVector(p1, p2, n2);
            }
        }

        private Vector3 PointLineVector(Vector3 p, Vector3 pLine, Vector3 nLine)  // TODO: move to library
        {
            // get vector from the specified point to a point on the line
            var v = pLine - p;

            // get the component of v orthogonal to a
            return VectorOrtho(v, nLine);
        }

        private Vector3 VectorOrtho(Vector3 v1, Vector3 v2)  // TODO: move to library
        {
            // get the component of v1 that is perpendicular to v2
            return v1 - Vector3.Dot(v1, v2) * v2;
        }
    }
}