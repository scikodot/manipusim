using MathNet.Numerics.LinearAlgebra;

namespace Logic.InverseKinematics
{
    using VectorFloat = Vector<float>;

    public class JacobianPseudoinverse : JacobianSolver
    {
        public JacobianPseudoinverse(float threshold, int maxIterations) : base(threshold, maxIterations) { }

        public static JacobianPseudoinverse Default()
        {
            return new JacobianPseudoinverse(_thresholdDefault, _maxIterationsDefault);
        }

        protected override VectorFloat GetCoordinateOffset(Matrix<float> jacobian, VectorFloat error)
        {
            // get Jacobian pseudoinverse
            var JP = jacobian.PseudoInverse();

            // calculate the displacement
            return -JP * error;  // TODO: check why minus should always be presented for dq in these methods!
        }
    }
}
