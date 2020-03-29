using System;
using OpenTK;

namespace Logic
{
    struct Quaternion
    {
        public Vector3 XYZ { get; }
        public float W { get; }

        public float X => XYZ.X;
        public float Y => XYZ.Y;
        public float Z => XYZ.Z;

        public static Quaternion Null => new Quaternion(0, 0, 0, 0);

        public static Quaternion Zero => new Quaternion(0, 0, 0, 1);

        public float Length => (float)Math.Sqrt(XYZ.LengthSquared + W * W);

        public Quaternion Conjugate => new Quaternion(-XYZ, W);

        public Quaternion(Vector3 xyz, float w)
        {
            XYZ = xyz;
            W = w;
        }

        public Quaternion(float x, float y, float z, float w)
        {
            XYZ = new Vector3(x, y, z);
            W = w;
        }

        public static Quaternion operator +(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q1.XYZ + q2.XYZ, q1.W + q2.W);
        }

        public static Quaternion operator -(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q1.XYZ - q2.XYZ, q1.W - q2.W);
        }

        public static Quaternion operator *(Quaternion q1, Quaternion q2)
        {
            return new Quaternion(q2.W * q1.XYZ + q1.W * q2.XYZ + Vector3.Cross(q1.XYZ, q2.XYZ), q1.W * q2.W - Vector3.Dot(q1.XYZ, q2.XYZ));
        }

        public static Quaternion operator *(Quaternion q, float s)
        {
            return new Quaternion(q.XYZ * s, q.W);
        }
    }

    struct DualQuaternion
    {
        public Quaternion real;
        public Quaternion dual;

        public DualQuaternion Conjugate => new DualQuaternion(real.Conjugate, dual.Conjugate);

        public static DualQuaternion Default => new DualQuaternion(Quaternion.Zero, Quaternion.Null);

        public float Length => (this * Conjugate).real.W;

        public Matrix4 Matrix
        {
            get
            {
                float w = real.W, w2 = real.W * real.W;
                float x = real.X, x2 = real.X * real.X;
                float y = real.Y, y2 = real.Y * real.Y;
                float z = real.Z, z2 = real.Z * real.Z;

                var rxx = w2 + x2 - y2 - z2;
                var rxy = 2 * x * y - 2 * w * z;
                var rxz = 2 * x * z + 2 * w * y;
                var ryx = 2 * x * y + 2 * w * z;
                var ryy = w2 - x2 + y2 - z2;
                var ryz = 2 * y * z - 2 * w * x;
                var rzx = 2 * x * z - 2 * w * y;
                var rzy = 2 * y * z + 2 * w * x;
                var rzz = w2 - x2 - y2 + z2;

                var dualNew = dual * real.Conjugate;

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
            this.real = real;
            this.dual = dual;
        }

        public DualQuaternion(Vector3 offset)
        {
            real = Quaternion.Zero;
            dual = new Quaternion(offset / 2, 0);
        }

        public DualQuaternion(Vector3 axis, float angle)
        {
            axis.Normalize();
            var cos = (float)Math.Cos(angle / 2);
            var sin = (float)Math.Sin(angle / 2);

            real = new Quaternion(axis * sin, cos);
            dual = Quaternion.Null;
        }

        public DualQuaternion(Vector3 axis, float angle, Vector3 offset)
        {
            axis.Normalize();
            var cos = (float)Math.Cos(angle / 2);
            var sin = (float)Math.Sin(angle / 2);

            real = new Quaternion(axis * sin, cos);
            dual = new Quaternion(offset / 2, 0) * real;
        }

        public DualQuaternion(Vector3 axis, Vector3 point, float angle)
        {
            axis.Normalize();
            var qR = new DualQuaternion(axis, angle);
            var qT = new DualQuaternion(point);
            var res = qT * qR * qT.Conjugate;

            real = res.real;
            dual = res.dual;
        }

        public DualQuaternion(Vector3 axis, Vector3 point, float angle, Vector3 offset)
        {
            axis.Normalize();
            var qR = new DualQuaternion(axis, angle, offset);
            var qT = new DualQuaternion(point);
            var res = qT * qR * qT.Conjugate;

            real = res.real;
            dual = res.dual;
        }

        public static DualQuaternion operator +(DualQuaternion q1, DualQuaternion q2)
        {
            return new DualQuaternion(q1.real + q2.real, q1.dual + q2.dual);
        }

        public static DualQuaternion operator -(DualQuaternion q1, DualQuaternion q2)
        {
            return new DualQuaternion(q1.real - q2.real, q1.dual - q2.dual);
        }

        public static DualQuaternion operator *(DualQuaternion q1, DualQuaternion q2)
        {
            return new DualQuaternion(q1.real * q2.real, q1.real * q2.dual + q1.dual * q2.real);
        }

        public DualQuaternion Normalized()
        {
            var fact = 1 / real.Length;
            return new DualQuaternion(real * fact, dual * fact);
        }
    }
}
