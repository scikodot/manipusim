using System;

using BulletSharp.Math;

namespace Logic
{
    public struct ImpDualQuat
    {
        public static readonly ImpDualQuat Zero = new ImpDualQuat(Quaternion.Identity, Vector3.Zero);  // TODO: make readonly ref?

        private Quaternion _real;
        public Quaternion Rotation => _real;  // TODO: make readonly ref?

        private Vector3 _dual;
        public Vector3 Translation => _dual;  // TODO: make readonly ref?

        public float Length => (this * Conjugate())._real.W;  // TODO: optimize?

        private ImpDualQuat(Quaternion real, Vector3 dual)
        {
            _real = real;
            _dual = dual;
        }

        public ImpDualQuat(Vector3 translation)
        {
            _real = Quaternion.Identity;
            _dual = translation;
        }

        public ImpDualQuat(Vector3 axis, float angle)
        {
            if (axis == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");

            var halfAngle = 0.5f * angle;
            var cos = (float)Math.Cos(halfAngle);
            var sin = (float)Math.Sin(halfAngle);

            _real = new Quaternion(Vector3.Normalize(axis) * sin, cos);
            _dual = Vector3.Zero;
        }

        public ImpDualQuat(Vector3 axis, float angle, Vector3 translation)
        {
            if (axis == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");

            var halfAngle = 0.5f * angle;
            var cos = (float)Math.Cos(halfAngle);
            var sin = (float)Math.Sin(halfAngle);

            _real = new Quaternion(Vector3.Normalize(axis) * sin, cos);
            _dual = translation;
        }

        public ImpDualQuat(Vector3 axis, Vector3 axisOffset, float angle)
        {
            if (axis == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");

            var res = new ImpDualQuat(axisOffset) * new ImpDualQuat(axis, angle) * new ImpDualQuat(-axisOffset);

            _real = res._real;
            _dual = res._dual;
        }

        // TODO: make more clear use; here, an ambiguity is presented - offset/rotate or rotate/offset? (each option looks weird anyway)
        public ImpDualQuat(Vector3 axis, Vector3 axisOffset, float angle, Vector3 translation)
        {
            if (axis == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");

            var res = new ImpDualQuat(axisOffset) * new ImpDualQuat(axis, angle) * new ImpDualQuat(-axisOffset) * new ImpDualQuat(translation);

            _real = res._real;
            _dual = res._dual;
        }

        public ImpDualQuat Normalized()
        {
            return new ImpDualQuat(Quaternion.Normalize(_real), _dual);
        }

        public ImpDualQuat Conjugate()
        {
            var conjugateReal = Quaternion.Conjugate(_real);
            return new ImpDualQuat(conjugateReal, -conjugateReal.Rotate(_dual));
        }

        public static ImpDualQuat Align(Vector3 vec1, Vector3 vec2)
        {
            if (vec1 == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");
            if (vec2 == Vector3.Zero) throw new ArgumentException("The rotation axis cannot be zero.", "axis");

            vec1 = Vector3.Normalize(vec1);
            vec2 = Vector3.Normalize(vec2);

            var alignAxis = Vector3.Cross(vec1, vec2);
            if (alignAxis == Vector3.Zero)
                return Zero;
            else
            {
                var alignAngle = (float)Math.Acos(Vector3.Dot(vec1, vec2));  // TODO: make check for success; Acos is prone to errors
                return new ImpDualQuat(alignAxis, alignAngle);
            }
        }

        public Vector3 Rotate(Vector3 v)
        {
            return _real.Rotate(v);
        }

        public Vector3 Translate(Vector3 v)
        {
            return v + _dual;
        }

        public Vector3 Transform(Vector3 v)
        {
            return _real.Rotate(v) + _dual;
        }

        public ImpDualQuat WithoutRotation()
        {
            return new ImpDualQuat(Quaternion.Identity, _dual);
        }

        public ImpDualQuat WithoutTranslation()
        {
            return new ImpDualQuat(_real, Vector3.Zero);
        }

        public OpenTK.Mathematics.Matrix4 ToMatrix()
        {
            return new OpenTK.Mathematics.Matrix4(_real.ToMatrix())
            {
                M41 = _dual.X,
                M42 = _dual.Y,
                M43 = _dual.Z
            };
        }

        public static ImpDualQuat operator +(ImpDualQuat q1, ImpDualQuat q2)
        {
            return new ImpDualQuat(q1._real + q2._real, q1._dual + q2._dual);
        }

        public static ImpDualQuat operator -(ImpDualQuat q1, ImpDualQuat q2)
        {
            return new ImpDualQuat(q1._real - q2._real, q1._dual - q2._dual);
        }

        public static ImpDualQuat operator *(ImpDualQuat q1, ImpDualQuat q2)
        {
            return new ImpDualQuat(q1._real * q2._real, q1._dual + q1._real.Rotate(q2._dual));
        }

        public override string ToString() => string.Format("R: [{0}], D: [{1}]", _real, _dual);
    }
}
