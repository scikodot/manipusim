using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;

using OpenTK.Graphics.OpenGL4;
using BulletSharp.Math;

using Graphics;
using System.Linq;

namespace Logic
{
    /// <summary>
    /// Represents a bezier curve with as many points as you want.
    /// </summary>
    [Serializable]
    public class BezierCurve : IDisposable
    {
        public class BezierCurveModel : IDisposable
        {
            private static int _pointsCount = 1000;
            public static ref int PointsCount => ref _pointsCount;

            private static Vector3 _color = Vector3.Zero;
            public static ref Vector3 Color => ref _color;  // TODO: use in GUI!

            public BezierCurve BezierCurve { get; }
            public Model Model { get; }

            public bool IsSetup => Model.IsSetup;

            public BezierCurveModel(BezierCurve bezierCurve)
            {
                BezierCurve = bezierCurve;

                // create an empty model with the specified material
                Model = new Model(new MeshVertex[_pointsCount], new uint[2 * _pointsCount],
                    new MeshMaterial { Diffuse = new OpenTK.Mathematics.Color4(_color.X, _color.Y, _color.Z, 1.0f) });

                float step = 1.0f / (_pointsCount - 1);
                var vertices = bezierCurve.CalculatePoints(step).Select(p => new MeshVertex { Position = new OpenTK.Mathematics.Vector3(p.X, p.Y, p.Z) });
                
                var indices = new List<uint>();
                for (uint i = 0; i < _pointsCount - 1; i++)
                {
                    indices.Add(i);
                    indices.Add(i + 1);
                }

                Model.Meshes[0].UpdateVertices(0, _pointsCount, vertices.ToArray());
                Model.Meshes[0].UpdateIndices(0, 2 * (_pointsCount - 1), indices.ToArray());
            }

            public void Render(Shader shader)
            {
                if (IsSetup)
                    Model.Render(shader, () =>
                    {
                        GL.DrawElements(BeginMode.Lines, 2 * (_pointsCount - 1), DrawElementsType.UnsignedInt, 0);
                    });
            }

            public void Dispose()
            {
                // clear managed resources
                Model.Dispose();

                // suppress finalization
                GC.SuppressFinalize(this);
            }
        }

        private readonly List<Vector3> _points;

        public BezierCurveModel Model { get; private set; }

        /// <summary>
        /// The parallel value.
        /// </summary>
        /// <remarks>
        /// This value defines whether the curve should be calculated as a
        /// parallel curve to the original bezier curve. A value of 0.0f represents
        /// the original curve, 5.0f i.e. stands for a curve that has always a distance
        /// of 5.0f to the orignal curve at any point.
        /// </remarks>
        public float Parallel;

        /// <summary>
        /// Gets the points of this curve.
        /// </summary>
        /// <remarks>The first point and the last points represent the anchor points.</remarks>
        public IList<Vector3> Points => _points;

        /// <summary>
        /// Initializes a new instance of the <see cref="BezierCurve"/> struct.
        /// </summary>
        /// <param name="points">The points.</param>
        public BezierCurve(IEnumerable<Vector3> points)
        {
            if (points == null)
            {
                throw new ArgumentNullException(nameof(points), "Must point to a valid list of points structures.");
            }

            _points = new List<Vector3>(points);
            Parallel = 0.0f;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="BezierCurve"/> struct.
        /// </summary>
        /// <param name="points">The points.</param>
        public BezierCurve(params Vector3[] points)
        {
            if (points == null)
            {
                throw new ArgumentNullException(nameof(points), "Must point to a valid list of points structures.");
            }

            _points = new List<Vector3>(points);
            Parallel = 0.0f;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="BezierCurve"/> struct.
        /// </summary>
        /// <param name="parallel">The parallel value.</param>
        /// <param name="points">The points.</param>
        public BezierCurve(float parallel, params Vector3[] points)
        {
            if (points == null)
            {
                throw new ArgumentNullException(nameof(points), "Must point to a valid list of points structures.");
            }

            Parallel = parallel;
            _points = new List<Vector3>(points);
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="BezierCurve"/> struct.
        /// </summary>
        /// <param name="parallel">The parallel value.</param>
        /// <param name="points">The points.</param>
        public BezierCurve(float parallel, IEnumerable<Vector3> points)
        {
            if (points == null)
            {
                throw new ArgumentNullException(nameof(points), "Must point to a valid list of points structures.");
            }

            Parallel = parallel;
            _points = new List<Vector3>(points);
        }

        public void SetModel()
        {
            Model = new BezierCurveModel(this);
        }

        /// <summary>
        /// Calculates the point with the specified t.
        /// </summary>
        /// <param name="t">The t value, between 0.0f and 1.0f.</param>
        /// <returns>Resulting point.</returns>
        [Pure]
        public Vector3 CalculatePoint(float t)
        {
            return CalculatePoint(_points, t, Parallel);
        }

        /// <summary>
        /// Discretize the curve with the specified step.
        /// </summary>
        /// <param name="step">A step size by the curve's parameter t.</param>
        /// <returns>A list of points on the curve.</returns>
        public List<Vector3> CalculatePoints(float step)
        {
            var points = new List<Vector3>();

            float counter = 0;
            while (counter <= 1)
            {
                points.Add(CalculatePoint(counter));
                counter += step;
            }

            return points;
        }

        /// <summary>
        /// Discretize the curve on the specified amount of points.
        /// </summary>
        /// <param name="step">An amount of points the discretized curve should contain.</param>
        /// <returns>A list of points on the curve.</returns>
        //public List<Vector3> CalculatePoints(int pointsCount)
        //{
        //    var points = new List<Vector3>();

        //    float step = 1.0f / (pointsCount - 1);
        //    for (int i = 0; i < pointsCount; i++)
        //    {
        //        points.Add(CalculatePoint(i * step));
        //    }

        //    return points;
        //}

        /// <summary>
        /// Calculates the length of this bezier curve.
        /// </summary>
        /// <param name="precision">The precision.</param>
        /// <returns>Length of curve.</returns>
        /// <remarks>
        /// The precision gets better as the <paramref name="precision"/>
        /// value gets smaller.
        /// </remarks>
        [Pure]
        public float CalculateLength(float precision)
        {
            return CalculateLength(_points, precision, Parallel);
        }

        /// <summary>
        /// Calculates the length of the specified bezier curve.
        /// </summary>
        /// <param name="points">The points.</param>
        /// <param name="precision">The precision value.</param>
        /// <returns>
        /// The precision gets better as the <paramref name="precision"/>
        /// value gets smaller.
        /// </returns>
        [Pure]
        public static float CalculateLength(IList<Vector3> points, float precision)
        {
            return CalculateLength(points, precision, 0.0f);
        }

        /// <summary>
        /// Calculates the length of the specified bezier curve.
        /// </summary>
        /// <param name="points">The points.</param>
        /// <param name="precision">The precision value.</param>
        /// <param name="parallel">The parallel value.</param>
        /// <returns>Length of curve.</returns>
        /// <remarks>
        ///  <para>
        /// The precision gets better as the <paramref name="precision"/>
        /// value gets smaller.
        ///  </para>
        ///  <para>
        /// The <paramref name="parallel"/> parameter defines whether the curve should be calculated as a
        /// parallel curve to the original bezier curve. A value of 0.0f represents
        /// the original curve, 5.0f represents a curve that has always a distance
        /// of 5.0f to the orignal curve.
        ///  </para>
        /// </remarks>
        [Pure]
        public static float CalculateLength(IList<Vector3> points, float precision, float parallel)
        {
            var length = 0.0f;
            var old = CalculatePoint(points, 0.0f, parallel);

            for (var i = precision; i < 1.0f + precision; i += precision)
            {
                var n = CalculatePoint(points, i, parallel);
                length += (n - old).Length;
                old = n;
            }

            return length;
        }

        /// <summary>
        /// Calculates the point on the given bezier curve with the specified t parameter.
        /// </summary>
        /// <param name="points">The points.</param>
        /// <param name="t">The t parameter, a value between 0.0f and 1.0f.</param>
        /// <returns>Resulting point.</returns>
        [Pure]
        public static Vector3 CalculatePoint(IList<Vector3> points, float t)
        {
            return CalculatePoint(points, t, 0.0f);
        }

        /// <summary>
        /// Calculates the point on the given bezier curve with the specified t parameter.
        /// </summary>
        /// <param name="points">The points.</param>
        /// <param name="t">The t parameter, a value between 0.0f and 1.0f.</param>
        /// <param name="parallel">The parallel value.</param>
        /// <returns>Resulting point.</returns>
        /// <remarks>
        /// The <paramref name="parallel"/> parameter defines whether the curve should be calculated as a
        /// parallel curve to the original bezier curve. A value of 0.0f represents
        /// the original curve, 5.0f represents a curve that has always a distance
        /// of 5.0f to the orignal curve.
        /// </remarks>
        [Pure]
        public static Vector3 CalculatePoint(IList<Vector3> points, float t, float parallel)
        {
            var r = default(Vector3);
            var c = 1.0d - t;
            float temp;
            var i = 0;

            foreach (var point in points)
            {
                temp = OpenTK.Mathematics.MathHelper.BinomialCoefficient(points.Count - 1, i) * 
                    (float)(Math.Pow(t, i) * Math.Pow(c, points.Count - 1 - i));

                r.X += temp * point.X;
                r.Y += temp * point.Y;
                r.Z += temp * point.Z;
                i++;
            }

            if (parallel == 0.0f)
            {
                return r;
            }

            // TODO: try to implement parallel offset below; offset in 3D is not unique, so it might be not possible
            return r;

            //Vector3 perpendicular;

            //if (t != 0.0f)
            //{
            //    perpendicular = r - CalculatePointOfDerivative(points, t);
            //}
            //else
            //{
            //    perpendicular = points[1] - points[0];
            //}

            //return r + (Vector3.Normalize(perpendicular).PerpendicularRight * parallel);
        }

        /// <summary>
        /// Calculates the point with the specified t of the derivative of the given bezier function.
        /// </summary>
        /// <param name="points">The points.</param>
        /// <param name="t">The t parameter, value between 0.0f and 1.0f.</param>
        /// <returns>Resulting point.</returns>
        [Pure]
        private static Vector3 CalculatePointOfDerivative(IList<Vector3> points, float t)
        {
            var r = default(Vector3);
            var c = 1.0d - t;
            float temp;
            var i = 0;

            foreach (var point in points)
            {
                temp = OpenTK.Mathematics.MathHelper.BinomialCoefficient(points.Count - 2, i) * 
                    (float)(Math.Pow(t, i) * Math.Pow(c, points.Count - 2 - i));

                r.X += temp * point.X;
                r.Y += temp * point.Y;
                r.Z += temp * point.Z;
                i++;
            }

            return r;
        }

        public void Dispose()
        {
            Model.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
