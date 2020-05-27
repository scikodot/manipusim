using MathNet.Numerics.LinearAlgebra;

namespace Logic.InverseKinematics
{
    using VectorFloat = Vector<float>;

    public class DampedLeastSquares : JacobianSolver
    {
        protected static float _dampingDefault = 1f;

        protected float _damping;
        public ref float Damping => ref _damping;

        public DampedLeastSquares(float threshold, int maxIterations, float damping) : base(threshold, maxIterations) 
        {
            _damping = damping;
        }

        public static DampedLeastSquares Default()
        {
            return new DampedLeastSquares(_thresholdDefault, _maxIterationsDefault, _dampingDefault);
        }

        protected override VectorFloat GetCoordinateOffset(Matrix<float> jacobian, VectorFloat error)
        {
            // get Jacobian transpose and an identity matrix
            var jacobianTranspose = jacobian.Transpose();
            var identity = Matrix<float>.Build.DenseIdentity(error.Count);

            // calculate the displacement
            return -jacobianTranspose * (jacobian * jacobianTranspose + _damping * _damping * identity).Solve(error);
        }
    }
}
