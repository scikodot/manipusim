using System.Numerics;

namespace Logic
{
    public static class Geometry
    {
        public static Vector3 LinePlaneIntersection(Vector3 pLine, Vector3 nLine, Vector3 pPlane, Vector3 nPlane)
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

        public static Vector3 VectorPlaneProjection(Vector3 v, Vector3 n)
        {
            // get the component of v that is perpendicular to plane's normal, i.e. lies in that plane
            return VectorOrtho(v, n);
        }

        //public static Vector3 LinesIntersection(Vector3 p1, Vector3 a1, Vector3 p2, Vector3 a2)  // TODO: try to optimize
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

        public static float PointLineDistance(Vector3 p, Vector3 pLine, Vector3 nLine)
        {
            return PointLineVector(p, pLine, nLine).Length();
        }

        public static float PointLineDistance(Vector2 p, Vector2 pLine, Vector2 nLine)
        {
            return PointLineVector(p, pLine, nLine).Length();
        }

        public static float LinesDistance(Vector3 p1, Vector3 n1, Vector3 p2, Vector3 n2)
        {
            return LinesVector(p1, n1, p2, n2).Length();
        }

        private static Vector3 LinesVector(Vector3 p1, Vector3 n1, Vector3 p2, Vector3 n2)
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

        private static Vector3 PointLineVector(Vector3 p, Vector3 pLine, Vector3 nLine)
        {
            // get vector from the specified point to a point on the line
            var v = pLine - p;

            // get the component of v orthogonal to a
            return VectorOrtho(v, nLine);
        }

        private static Vector2 PointLineVector(Vector2 p, Vector2 pLine, Vector2 nLine)
        {
            // get vector from the specified point to a point on the line
            var v = pLine - p;

            // get the component of v orthogonal to a
            return VectorOrtho(v, nLine);
        }

        private static  Vector3 VectorOrtho(Vector3 v1, Vector3 v2)
        {
            // get the component of v1 that is perpendicular to v2
            return v1 - Vector3.Dot(v1, v2) * v2;
        }

        private static Vector2 VectorOrtho(Vector2 v1, Vector2 v2)
        {
            // get the component of v1 that is perpendicular to v2
            return v1 - Vector2.Dot(v1, v2) * v2;
        }
    }
}
