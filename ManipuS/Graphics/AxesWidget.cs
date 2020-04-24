using Graphics;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;

namespace Graphics
{
    public class AxesWidget
    {
        public Axis axisX, axisY, axisZ;
        public IRenderable Parent;

        public AxesWidget()
        {
            axisX = new Axis(Vector4.UnitW, new Vector4(1, 0, 0, 1), new Vector4(1, 0, 0, 1));
            axisY = new Axis(Vector4.UnitW, new Vector4(0, 1, 0, 1), new Vector4(0, 1, 0, 1));
            axisZ = new Axis(Vector4.UnitW, new Vector4(0, 0, 1, 1), new Vector4(0, 0, 1, 1));
        }

        public void Attach(IRenderable renderable)
        {
            Parent = renderable;
        }

        public void Transform(Matrix4 view, Matrix4 proj, Vector2 posCurr, MouseState stateCurr)
        {
            var parentState = Parent.State;
            axisX.Transform(ref parentState, view, proj, posCurr, stateCurr);  // TODO: prioritize axes polling, so that those with smaller depth go first
            axisY.Transform(ref parentState, view, proj, posCurr, stateCurr);
            axisZ.Transform(ref parentState, view, proj, posCurr, stateCurr);
            Parent.State = parentState;
        }
    }

    public class Axis
    {
        public Vector4 Start, End;
        public Vector2 StartNDC, EndNDC;
        public PlainModel Model;
        public float CursorDist;

        public bool FirstClick = true, Started;
        public Vector3 Offset;

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

        public bool Poll(Matrix4 view, Matrix4 proj, Vector2 cursorPos)  // TODO: optimize, refactor
        {
            // transform start point to NDC
            var startView = Start * view;
            var startProj = startView * proj;
            startProj /= startProj.W;
            StartNDC = new Vector2(startProj.X, startProj.Y);

            // transform end point to NDC
            var endView = End * view;
            var endProj = endView * proj;
            endProj /= endProj.W;
            EndNDC = new Vector2(endProj.X, endProj.Y);

            CursorDist = Vector2.Distance(cursorPos, EndNDC);
            return CursorDist < 0.1f;
        }

        public void Transform(ref Matrix4 state, Matrix4 view, Matrix4 proj, Vector2 posCurr, MouseState stateCurr)  // TODO: optimize, remove state
        {
            var axisActive = Poll(view, proj, posCurr);
            //Console.SetCursorPosition(0, 10);
            //Console.WriteLine($"Axis X is {axisActive}");
            //Console.WriteLine($"Mouse button is {stateCurr.RightButton}");

            if ((axisActive || Started) && stateCurr.RightButton == ButtonState.Pressed)
            {
                Started = true;

                // find the new mouse point projection onto the X axis (in NDC space)
                var mNew = posCurr;  //new Vector2(stateCurr.X, stateCurr.Y);
                var a = EndNDC - StartNDC;
                var n = mNew - EndNDC;
                var v = a.Normalized() * Vector2.Dot(a, n);

                // cast a ray from the near plane to the far plane
                var raycastRes = Raycast(EndNDC + v, view, proj);

                // find the point of intersection of the ray and the XY plane (in World space)
                var p0 = Vector3.Zero;  // a point on the XY plane
                var l0 = raycastRes.Item1.Xyz;  // a point on the near plane
                var norm = Vector3.UnitZ;  // the normal of the XY plane
                var l = raycastRes.Item2.Xyz.Normalized();  // a direction of the ray

                var nom = Vector3.Dot(p0 - l0, norm);
                var denom = Vector3.Dot(l, norm);

                Vector3 intersection = default;
                if (denom != 0)
                {
                    // a unique point of intersection
                    var d = nom / denom;
                    intersection = l0 + l * d;
                }
                else
                {
                    if (nom == 0)
                    {
                        // the ray is on the plane, i.e. infinite amount of intersections

                    }
                    else
                    {
                        // the ray is parallel to the plane, i.e. no intersections

                    }
                }

                if (FirstClick)
                {
                    Offset = new Vector3(intersection.X, 0, 0) - End.Xyz;
                    FirstClick = false;
                }
                else
                {
                    // project the intersecton point onto the X axis (in World space)
                    Vector3 translation = new Vector3(intersection.X, 0, 0) - End.Xyz - Offset;

                    Start += new Vector4(translation);
                    End += new Vector4(translation);

                    state.M14 += translation.X;
                    Model.State = state;  // TODO: move to separate method UpdateState() ?
                }
            }
            else
            {
                Started = false;
                FirstClick = true;
            }
        }

        public (Vector4, Vector4) Raycast(Vector2 ndc, Matrix4 view, Matrix4 proj)  // TODO: optimize
        {
            var vStart = new Vector4(ndc.X, ndc.Y, -1, 1);
            var vEnd = new Vector4(ndc.X, ndc.Y, 0, 1);

            var projInv = Matrix4.Invert(Matrix4.Transpose(proj));
            var viewInv = Matrix4.Invert(Matrix4.Transpose(view));

            var rayStartCamera = projInv * vStart;
            rayStartCamera /= rayStartCamera.W;
            var rayStartWorld = viewInv * rayStartCamera;
            rayStartWorld /= rayStartWorld.W;

            var rayEndCamera = projInv * vEnd;
            rayEndCamera /= rayEndCamera.W;
            var rayEndWorld = viewInv * rayEndCamera;
            rayEndWorld /= rayEndWorld.W;

            var rayDir = Vector4.Normalize(rayEndWorld - rayStartWorld);

            return (rayStartWorld, rayDir);
        }
    }
}
