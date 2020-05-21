using System.Numerics;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public class JacobianPseudoinverse : InverseKinematicsSolver
    {
        public JacobianPseudoinverse(float threshold, float stepSize, int maxTime) : base(threshold, stepSize, maxTime) { }

        public override (bool, int, float, VectorFloat) Execute(Manipulator agent, Vector3 goal, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = agent.Joints.Length - 1;

            VectorFloat initConfig = agent.q, dq;
            int iters = 0;
            while (iters++ < _maxTime)
            {
                // get positional/orientational error
                var error = GetError(agent, goal, joint);  // TODO: check for oscillations (the error starts increasing) and break if they appear

                // get Jacobian and its Moore-Penrose inverse, aka pseudoinverse
                var J = Jacobian.Create(agent, joint);
                var JP = J.PseudoInverse();

                // calculate the displacement
                dq = -JP * error;  // TODO: check why minus should always be presented for dq in these methods!

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
