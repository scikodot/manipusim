using MathNet.Numerics.LinearAlgebra;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

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
        //        // get Jacobian and its Moore-Penrose inverse, aka pseudoinverse
        //        var J = MathNet.Numerics.LinearAlgebra.Matrix<float>.Build.Dense(1, 1);  //JacobianSolver.CreateJacobian(agent, joint);
        //        var JP = J.PseudoInverse();

        //        // calculate the displacement
        //        dq = -JP * error;  // TODO: check why minus should always be presented for dq in these methods!

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
