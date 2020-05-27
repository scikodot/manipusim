using MathNet.Numerics.LinearAlgebra;
using System.Numerics;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

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

        protected override VectorFloat GetCoordinateOffset(Matrix<float> jacobian, VectorFloat error)
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

        //public override InverseKinematicsResult Execute(Manipulator agent, Vector3 goal, int joint = -1)
        //{
        //    // use gripper if default joint
        //    if (joint == -1)
        //        joint = agent.Joints.Length - 1;

        //    float alpha = _damping;
        //    VectorFloat configuration = agent.q, dq;
        //    VectorFloat error;

        //    // TODO: check for oscillations (the error starts increasing) and break if they appear
        //    int iterations = 0;
        //    while (ErrorExceedsThreshold(agent, configuration, goal, joint, _threshold, out error) && iterations++ < _maxIterations)
        //    {
        //        // get Jacobian, its transpose and an identity matrix
        //        var J = MathNet.Numerics.LinearAlgebra.Matrix<float>.Build.Dense(1, 1);  //JacobianSolver.CreateJacobian(agent, joint);
        //        var JT = J.Transpose();

        //        // compute damping coefficient
        //        var JJTe = J * JT * error;
        //        float nom = error.DotProduct(JJTe);
        //        float denom = JJTe.DotProduct(JJTe);
        //        if (denom != 0)
        //            alpha = nom / denom;

        //        // calculate the displacement
        //        dq = -alpha * JT * error;

        //        // update manipulator's configuration
        //        configuration = configuration.AddSubVector(dq);
        //    }

        //    var dist = agent.Joints[joint].Position.DistanceTo(goal);

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
