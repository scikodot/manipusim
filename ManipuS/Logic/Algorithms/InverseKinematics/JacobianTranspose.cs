using System.Numerics;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public class JacobianTranspose : InverseKinematicsSolver
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

        public override (bool, int, float, VectorFloat) Execute(Manipulator agent, Vector3 goal, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = agent.Joints.Length - 1;

            float alpha = _damping;
            VectorFloat initConfig = agent.q, dq;
            int iters = 0;
            while (iters++ < _maxIterations)
            {
                // get positional/orientational error
                var error = GetError(agent, goal, joint);  // TODO: check for oscillations (the error starts increasing) and break if they appear

                // get Jacobian, its transpose and an identity matrix
                var J = Jacobian.Create(agent, joint);
                var JT = J.Transpose();

                // compute damping coefficient
                var JJTe = J * JT * error;
                float nom = error.DotProduct(JJTe);
                float denom = JJTe.DotProduct(JJTe);
                if (denom != 0)
                    alpha = nom / denom;

                // calculate the displacement
                dq = -alpha * JT * error;

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
