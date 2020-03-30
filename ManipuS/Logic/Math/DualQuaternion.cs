using System;
using OpenTK;

namespace Logic
{
    public struct DualQuaternion
    {
        public Quaternion Real;
        public Quaternion Dual;

        public DualQuaternion Conjugate => new DualQuaternion(Real.Conjugate, Dual.Conjugate);

        public static DualQuaternion Default => new DualQuaternion(Quaternion.Zero, Quaternion.Null);

        public float Length => (this * Conjugate).Real.W;

        public Matrix4 Matrix
        {
            get
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

                return new Matrix4(
                    new Vector4(rxx, rxy, rxz, tx),
                    new Vector4(ryx, ryy, ryz, ty),
                    new Vector4(rzx, rzy, rzz, tz),
                    new Vector4(0, 0, 0, 1)
                );
            }
        }

        private DualQuaternion(Quaternion real, Quaternion dual)
        {
            Real = real;
            Dual = dual;
        }

        public DualQuaternion(Vector3 offset)
        {
            Real = Quaternion.Zero;
            Dual = new Quaternion(offset / 2, 0);
        }

        public DualQuaternion(Vector3 axis, float angle)
        {
            axis = axis.Normalized;
            var cos = (float)Math.Cos(angle / 2);
            var sin = (float)Math.Sin(angle / 2);

            Real = new Quaternion(axis * sin, cos);
            Dual = Quaternion.Null;
        }

        public DualQuaternion(Vector3 axis, float angle, Vector3 offset)
        {
            axis = axis.Normalized;
            var cos = (float)Math.Cos(angle / 2);
            var sin = (float)Math.Sin(angle / 2);

            Real = new Quaternion(axis * sin, cos);
            Dual = new Quaternion(offset / 2, 0) * Real;
        }

        public DualQuaternion(Vector3 axis, Vector3 Vector3, float angle)
        {
            axis = axis.Normalized;
            var qR = new DualQuaternion(axis, angle);
            var qT = new DualQuaternion(Vector3);
            var res = qT * qR * qT.Conjugate;

            Real = res.Real;
            Dual = res.Dual;
        }

        public DualQuaternion(Vector3 axis, Vector3 Vector3, float angle, Vector3 offset)
        {
            axis = axis.Normalized;
            var qR = new DualQuaternion(axis, angle, offset);
            var qT = new DualQuaternion(Vector3);
            var res = qT * qR * qT.Conjugate;

            Real = res.Real;
            Dual = res.Dual;
        }

        public static DualQuaternion operator +(DualQuaternion q1, DualQuaternion q2)
        {
            return new DualQuaternion(q1.Real + q2.Real, q1.Dual + q2.Dual);
        }

        public static DualQuaternion operator -(DualQuaternion q1, DualQuaternion q2)
        {
            return new DualQuaternion(q1.Real - q2.Real, q1.Dual - q2.Dual);
        }

        public static DualQuaternion operator *(DualQuaternion q1, DualQuaternion q2)
        {
            return new DualQuaternion(q1.Real * q2.Real, q1.Real * q2.Dual + q1.Dual * q2.Real);
        }

        public DualQuaternion Normalized()
        {
            var fact = 1 / Real.Length;
            return new DualQuaternion(Real * fact, Dual * fact);
        }

        public override string ToString() => string.Format("R: [{0}], D: [{1}]", Real, Dual);
    }
}
