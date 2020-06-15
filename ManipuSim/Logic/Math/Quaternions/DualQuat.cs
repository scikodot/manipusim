using System;
using System.Numerics;

namespace Logic
{
    public struct DualQuat
    {
        public static readonly DualQuat Zero = new DualQuat(Quaternion.Identity, new Quaternion(0, 0, 0, 0));

        private Quaternion _real;
        public Quaternion Rotation => _real;

        private Quaternion _dual;
        public Vector3 Translation => 2 * (_dual * Quaternion.Conjugate(_real)).XYZ();

        public float Length => (this * Conjugate())._real.W;

        private DualQuat(Quaternion real, Quaternion dual)
        {
            _real = real;
            _dual = dual;
        }

        public DualQuat(Vector3 translation)
        {
            _real = Quaternion.Identity;
            _dual = new Quaternion(0.5f * translation, 0);
        }

        public DualQuat(Vector3 axis, float angle)
        {
            if (axis == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");

            var halfAngle = 0.5f * angle;
            var cos = (float)Math.Cos(halfAngle);
            var sin = (float)Math.Sin(halfAngle);

            _real = new Quaternion(Vector3.Normalize(axis) * sin, cos);
            _dual = new Quaternion(0, 0, 0, 0);
        }

        public DualQuat(Vector3 axis, float angle, Vector3 translation)
        {
            if (axis == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");

            var halfAngle = 0.5f * angle;
            var cos = (float)Math.Cos(halfAngle);
            var sin = (float)Math.Sin(halfAngle);

            _real = new Quaternion(Vector3.Normalize(axis) * sin, cos);
            _dual = new Quaternion(0.5f * translation, 0) * _real;  // TODO: perhaps multiply in reverse order?
        }

        public DualQuat(Vector3 axis, Vector3 axisOffset, float angle)
        {
            if (axis == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");

            var res = new DualQuat(axisOffset) * new DualQuat(axis, angle) * new DualQuat(-axisOffset);

            _real = res._real;
            _dual = res._dual;
        }

        public DualQuat(Vector3 axis, Vector3 axisOffset, float angle, Vector3 translation)
        {
            if (axis == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");

            var res = new DualQuat(axisOffset) * new DualQuat(axis, angle) * new DualQuat(-axisOffset) * new DualQuat(translation);

            _real = res._real;
            _dual = res._dual;
        }

        public DualQuat Normalized()
        {
            var fact = 1 / _real.Length();
            return new DualQuat(_real * fact, _dual * fact);
        }

        public DualQuat Conjugate()
        {
            return new DualQuat(Quaternion.Conjugate(_real), Quaternion.Conjugate(_dual));
        }

        public OpenToolkit.Mathematics.Matrix4 ToMatrix()
        {
            var translation = Translation;
            return new OpenToolkit.Mathematics.Matrix4(_real.ToMatrix())
            {
                M41 = translation.X,
                M42 = translation.Y,
                M43 = translation.Z
            };
        }

        public static DualQuat operator +(DualQuat q1, DualQuat q2)
        {
            return new DualQuat(q1._real + q2._real, q1._dual + q2._dual);
        }

        public static DualQuat operator -(DualQuat q1, DualQuat q2)
        {
            return new DualQuat(q1._real - q2._real, q1._dual - q2._dual);
        }

        public static DualQuat operator *(DualQuat q1, DualQuat q2)
        {
            return new DualQuat(q1._real * q2._real, q1._real * q2._dual + q1._dual * q2._real);
        }

        public override string ToString() => string.Format("R: [{0}], D: [{1}]", _real, _dual);
    }
}
