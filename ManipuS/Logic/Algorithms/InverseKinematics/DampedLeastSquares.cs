using MathNet.Numerics.LinearAlgebra;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

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

        //public override InverseKinematicsResult Execute(Manipulator agent, Vector3 goal, int joint = -1)
        //{
        //    // use gripper if default joint
        //    if (joint == -1)
        //        joint = agent.Joints.Length - 1;

        //    VectorFloat configuration = agent.q, dq;
        //    VectorFloat error;

        //    // TODO: check for oscillations (the error starts increasing) and break if they appear
        //    int iterations = 0;
        //    while (ErrorExceedsThreshold(agent, configuration, goal, joint, _threshold, out error) && iterations++ < _maxIterations)
        //    {
        //        // get Jacobian, its transpose and an identity matrix
        //        var J = JacobianSolver.CreateJacobian(agent, joint);
        //        var JT = J.Transpose();
        //        var I = Matrix<float>.Build.DenseIdentity(error.Count);

        //        // calculate the displacement
        //        dq = -JT * (J * JT + _damping * _damping * I).Solve(error);

        //        // update manipulator's configuration
        //        configuration = configuration.AddSubVector(dq);
        //    }

        //    return new InverseKinematicsResult
        //    {
        //        Converged = true/*iterations > _maxIterations*/,
        //        Iterations = iterations - 1,
        //        Configuration = configuration,
        //        Error = error
        //    };
        //}
    }
}
