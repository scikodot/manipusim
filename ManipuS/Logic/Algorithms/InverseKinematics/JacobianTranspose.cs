using System.Numerics;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public class JacobianTranspose : InverseKinematicsSolver
    {
        private float _alpha = 0.1f;
        public ref float Alpha => ref _alpha;

        public JacobianTranspose(float precision, float stepSize, int maxTime) : base(precision, stepSize, maxTime) { }

        public override (bool, float, VectorFloat, bool[]) Execute(Obstacle[] obstacles, Manipulator agent, Vector3 goal, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = agent.Joints.Length - 1;

            float alpha = _alpha;
            VectorFloat initConfig = agent.q, dq;
            for (int j = 0; j < _maxTime; j++)
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

                // update maipulator's configuration
                agent.q = agent.q.AddSubVector(dq);
            }

            // checking for collisions of the found configuration
            bool[] collisions = DetectCollisions(agent, obstacles);
            var dist = agent.Joints[joint].Position.DistanceTo(goal);

            return (true, dist, agent.q - initConfig, collisions);
        }
    }
}
