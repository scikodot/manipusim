using System;
using System.Numerics;

namespace Logic
{
    public struct ImpDualQuat
    {
        public Quaternion Real;
        public Vector3 Dual;

        public static ImpDualQuat Zero => new ImpDualQuat(Quaternion.Zero, Vector3.Zero);

        public float Length => (this * Conjugate).Real.W;

        public ImpDualQuat Normalized => new ImpDualQuat(Real.Normalized, Dual);

        public ImpDualQuat Conjugate
        {
            get
            {
                var conjugateReal = Real.Conjugate;
                return new ImpDualQuat(conjugateReal, -conjugateReal.Transform(Dual));
            }
        }

        public Quaternion Rotation => Real;

        public Vector3 Translation => Dual;

        private ImpDualQuat(Quaternion real, Vector3 dual)
        {
            Real = real;
            Dual = dual;
        }

        public ImpDualQuat(Vector3 offset)
        {
            Real = Quaternion.Zero;
            Dual = offset;
        }

        public ImpDualQuat(Vector3 axis, float angle)
        {
            if (axis != Vector3.Zero)
                axis = Vector3.Normalize(axis);
            var cos = (float)Math.Cos(angle / 2);
            var sin = (float)Math.Sin(angle / 2);

            Real = new Quaternion(axis * sin, cos);
            Dual = Vector3.Zero;
        }

        public ImpDualQuat(Vector3 axis, float angle, Vector3 offset)
        {
            if (axis != Vector3.Zero)
                axis = Vector3.Normalize(axis);
            var cos = (float)Math.Cos(angle / 2);
            var sin = (float)Math.Sin(angle / 2);

            Real = new Quaternion(axis * sin, cos);
            Dual = offset;
        }

        public ImpDualQuat(Vector3 axis, Vector3 point, float angle)
        {
            if (axis != Vector3.Zero)
                axis = Vector3.Normalize(axis);
            var res = new ImpDualQuat(point) * new ImpDualQuat(axis, angle) * new ImpDualQuat(-point);

            Real = res.Real;
            Dual = res.Dual;
        }

        public ImpDualQuat(Vector3 axis, Vector3 currPoint, Vector3 targetPoint, float angle)
        {
            if (axis != Vector3.Zero)
                axis = Vector3.Normalize(axis);
            var toPoint = new ImpDualQuat(targetPoint - currPoint);
            var rotate = new ImpDualQuat(axis, angle);
            var fromPoint = new ImpDualQuat(-(targetPoint - currPoint));
            var one = toPoint * rotate;
            var two = one * fromPoint;
            var res = two;
            //var res = new ImpDualQuat(targetPoint - currPoint) * new ImpDualQuat(axis, angle) * new ImpDualQuat(-(targetPoint - currPoint));

            Real = res.Real;
            Dual = res.Dual;
        }

        public ImpDualQuat(ImpDualQuat currState, Vector3 axis, Vector3 currPoint, Vector3 targetPoint, float angle)
        {
            if (axis != Vector3.Zero)
                axis = Vector3.Normalize(axis);
            var toPoint = new ImpDualQuat(currState.Rotate(targetPoint - currPoint));
            var rotate = new ImpDualQuat(axis, angle);
            var fromPoint = new ImpDualQuat(currState.Rotate(-(targetPoint - currPoint)));
            var one = toPoint * rotate;
            var two = one * fromPoint;
            var res = two;
            //var res = new ImpDualQuat(targetPoint - currPoint) * new ImpDualQuat(axis, angle) * new ImpDualQuat(-(targetPoint - currPoint));

            Real = res.Real;
            Dual = res.Dual;
        }

        // TODO: make more clear use; here, an ambiguity is presented - offset/rotate or rotate/offset? (each option looks weird anyway)
        public ImpDualQuat(Vector3 axis, Vector3 point, float angle, Vector3 offset)
        {
            if (axis != Vector3.Zero)
                axis = Vector3.Normalize(axis);
            var res = new ImpDualQuat(offset) * new ImpDualQuat(point - offset) * new ImpDualQuat(axis, angle) * new ImpDualQuat(-(point + offset));

            Real = res.Real;
            Dual = res.Dual;
        }

        public static ImpDualQuat Align(Vector3 axis1, Vector3 axis2)
        {
            if (axis1 != Vector3.Zero)
                axis1 = Vector3.Normalize(axis1);

            if (axis2 != Vector3.Zero)
                axis2 = Vector3.Normalize(axis2);

            var alignAxis = Vector3.Cross(axis1, axis2);
            if (alignAxis != Vector3.Zero)
                alignAxis = Vector3.Normalize(alignAxis);

            var alignAngle = (float)Math.Acos(Vector3.Dot(axis1, axis2));  // TODO: make check for success; Acos is prone to errors
            return new ImpDualQuat(alignAxis, alignAngle);
        }

        public Vector3 Rotate(Vector3 v)
        {
            return Real.Transform(v);
        }

        public Vector3 Translate(Vector3 v)
        {
            return v + Dual;
        }

        public Vector3 Transform(Vector3 v)
        {
            return Real.Transform(v) + Dual;
        }

        public Matrix4 ToMatrix(bool transpose = false)
        {
            float w = Real.W, w2 = Real.W * Real.W;
            float x = Real.X, x2 = Real.X * Real.X;
            float y = Real.Y, y2 = Real.Y * Real.Y;
            float z = Real.Z, z2 = Real.Z * Real.Z;

            var rxx = w2 + x2 - y2 - z2;
            var rxy = 2 * x * y - 2 * w * z;
            var rxz = 2 * x * z + 2 * w * y;
            var ryx = 2 * x * y + 2 * w * z;
            var ryy = w2 - x2 + y2 - z2;
            var ryz = 2 * y * z - 2 * w * x;
            var rzx = 2 * x * z - 2 * w * y;
            var rzy = 2 * y * z + 2 * w * x;
            var rzz = w2 - x2 - y2 + z2;

            if (transpose)
                return new Matrix4(
                    new Vector4(rxx, ryx, rzx, 0),
                    new Vector4(rxy, ryy, rzy, 0),
                    new Vector4(rxz, ryz, rzz, 0),
                    new Vector4(Dual, 1)
                );
            else
                return new Matrix4(
                    new Vector4(rxx, rxy, rxz, Dual.X),
                    new Vector4(ryx, ryy, ryz, Dual.Y),
                    new Vector4(rzx, rzy, rzz, Dual.Z),
                    new Vector4(0, 0, 0, 1)
                );
        }

        public BulletSharp.Math.Matrix ToBulletMatrix(bool transpose = false)
        {
            float w = Real.W, w2 = Real.W * Real.W;
            float x = Real.X, x2 = Real.X * Real.X;
            float y = Real.Y, y2 = Real.Y * Real.Y;
            float z = Real.Z, z2 = Real.Z * Real.Z;

            var rxx = w2 + x2 - y2 - z2;
            var rxy = 2 * x * y - 2 * w * z;
            var rxz = 2 * x * z + 2 * w * y;
            var ryx = 2 * x * y + 2 * w * z;
            var ryy = w2 - x2 + y2 - z2;
            var ryz = 2 * y * z - 2 * w * x;
            var rzx = 2 * x * z - 2 * w * y;
            var rzy = 2 * y * z + 2 * w * x;
            var rzz = w2 - x2 - y2 + z2;

            if (!transpose)
                return new BulletSharp.Math.Matrix(
                    rxx, ryx, rzx, 0,
                    rxy, ryy, rzy, 0,
                    rxz, ryz, rzz, 0,
                    Dual.X, Dual.Y, Dual.Z, 1
                );
            else
                return new BulletSharp.Math.Matrix(
                    rxx, rxy, rxz, Dual.X,
                    ryx, ryy, ryz, Dual.Y,
                    rzx, rzy, rzz, Dual.Z,
                    0, 0, 0, 1
                );
        }

        public static ImpDualQuat operator +(ImpDualQuat q1, ImpDualQuat q2)
        {
            return new ImpDualQuat(q1.Real + q2.Real, q1.Dual + q2.Dual);
        }

        public static ImpDualQuat operator -(ImpDualQuat q1, ImpDualQuat q2)
        {
            return new ImpDualQuat(q1.Real - q2.Real, q1.Dual - q2.Dual);
        }

        public static ImpDualQuat operator *(ImpDualQuat q1, ImpDualQuat q2)
        {
            return new ImpDualQuat(q1.Real * q2.Real, q1.Dual + q1.Real.Transform(q2.Dual));
        }

        public override string ToString() => string.Format("R: [{0}], D: [{1}]", Real, Dual);
    }
}
