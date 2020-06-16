using MathNet.Numerics.LinearAlgebra;

namespace Logic.InverseKinematics
{
    public class JacobianTranspose : JacobianSolver
    {
        protected static float _dampingDefault = 0.1f;

        protected float _damping;
        public ref float Damping => ref _damping;

        public JacobianTranspose(float threshold, int maxIterations, float damping) : base(threshold, maxIterations)
        {
            _damping = damping;
        }

        public static JacobianTranspose Default()
        {
            return new JacobianTranspose(_thresholdDefault, _maxIterationsDefault, _dampingDefault);
        }

        protected override Vector<float> GetCoordinateOffset(Matrix<float> jacobian, Vector<float> error)
        {
            // get Jacobian transpose
            var jacobianTranspose = jacobian.Transpose();

            // compute damping coefficient
            var JJTe = jacobian * jacobianTranspose * error;
            float nom = error.DotProduct(JJTe);
            float denom = JJTe.DotProduct(JJTe);
            if (denom != 0)
                _damping = nom / denom;

            // calculate the displacement
            return -_damping * jacobianTranspose * error;
        }
    }
}
