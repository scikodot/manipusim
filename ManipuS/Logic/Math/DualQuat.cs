using System;

namespace Logic
{
    public struct DualQuat
    {
        public Quaternion Real;
        public Quaternion Dual;

        public static DualQuat Zero => new DualQuat(Quaternion.Zero, Quaternion.Null);

        public float Length => (this * Conjugate).Real.W;

        public DualQuat Normalized
        {
            get
            {
                var fact = 1 / Real.Length;
                return new DualQuat(Real * fact, Dual * fact);
            }
        }

        public DualQuat Conjugate => new DualQuat(Real.Conjugate, Dual.Conjugate);

        public Quaternion Rotation => Real;

        public Vector3 Translation => 2 * (Dual * Real.Conjugate).XYZ;

        private DualQuat(Quaternion real, Quaternion dual)
        {
            Real = real;
            Dual = dual;
        }

        public DualQuat(Vector3 offset)
        {
            Real = Quaternion.Zero;
            Dual = new Quaternion(offset / 2, 0);
        }

        public DualQuat(Vector3 axis, float angle)
        {
            axis = axis.Normalized;
            var cos = (float)Math.Cos(angle / 2);
            var sin = (float)Math.Sin(angle / 2);

            Real = new Quaternion(axis * sin, cos);
            Dual = Quaternion.Null;
        }

        public DualQuat(Vector3 axis, float angle, Vector3 offset)
        {
            axis = axis.Normalized;
            var cos = (float)Math.Cos(angle / 2);
            var sin = (float)Math.Sin(angle / 2);

            Real = new Quaternion(axis * sin, cos);
            Dual = new Quaternion(offset / 2, 0) * Real;
        }

        public DualQuat(Vector3 axis, Vector3 Vector3, float angle)
        {
            axis = axis.Normalized;
            var qR = new DualQuat(axis, angle);
            var qT = new DualQuat(Vector3);
            var res = qT * qR * qT.Conjugate;

            Real = res.Real;
            Dual = res.Dual;
        }

        public DualQuat(Vector3 axis, Vector3 point, float angle, Vector3 offset)
        {
            axis = axis.Normalized;
            var qR = new DualQuat(axis, angle, offset);
            var qT = new DualQuat(point);
            var res = qT * qR * qT.Conjugate;

            Real = res.Real;
            Dual = res.Dual;
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

            var dualNew = Dual * Real.Conjugate;

            var tx = 2 * dualNew.X;
            var ty = 2 * dualNew.Y;
            var tz = 2 * dualNew.Z;

            if (transpose)
                return new Matrix4(
                    new Vector4(rxx, ryx, rzx, 0),
                    new Vector4(rxy, ryy, rzy, 0),
                    new Vector4(rxz, ryz, rzz, 0),
                    new Vector4(tx, ty, tz, 1)
                );
            else
                return new Matrix4(
                    new Vector4(rxx, rxy, rxz, tx),
                    new Vector4(ryx, ryy, ryz, ty),
                    new Vector4(rzx, rzy, rzz, tz),
                    new Vector4(0, 0, 0, 1)
                );
        }

        public static DualQuat operator +(DualQuat q1, DualQuat q2)
        {
            return new DualQuat(q1.Real + q2.Real, q1.Dual + q2.Dual);
        }

        public static DualQuat operator -(DualQuat q1, DualQuat q2)
        {
            return new DualQuat(q1.Real - q2.Real, q1.Dual - q2.Dual);
        }

        public static DualQuat operator *(DualQuat q1, DualQuat q2)
        {
            return new DualQuat(q1.Real * q2.Real, q1.Real * q2.Dual + q1.Dual * q2.Real);
        }

        public override string ToString() => string.Format("R: [{0}], D: [{1}]", Real, Dual);
    }
}
