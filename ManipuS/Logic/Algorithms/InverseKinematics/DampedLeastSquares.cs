using System.Numerics;

using MathNet.Numerics.LinearAlgebra;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public class DampedLeastSquares : InverseKinematicsSolver
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

        public override (bool, int, float, VectorFloat) Execute(Manipulator agent, Vector3 goal, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = agent.Joints.Length - 1;

            VectorFloat initConfig = agent.q, dq;
            int iters = 0;
            while (iters++ < _maxIterations)
            {
                // get positional/orientational error
                var error = GetError(agent, goal, joint);  // TODO: check for oscillations (the error starts increasing) and break if they appear

                // get Jacobian, its transpose and an identity matrix
                var J = Jacobian.Create(agent, joint);
                var JT = J.Transpose();
                var I = Matrix<float>.Build.DenseIdentity(error.Count);

                // calculate the displacement
                dq = -JT * (J * JT + _damping * _damping * I).Solve(error);

                // update manipulator's configuration
                agent.q = agent.q.AddSubVector(dq);

                if (agent.GripperPos.DistanceTo(goal) < _threshold)
                    break;
            }

            var dist = agent.Joints[joint].Position.DistanceTo(goal);

            return (true, iters - 1, dist, agent.q - initConfig);
        }
    }
}
