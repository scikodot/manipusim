using System.Numerics;

using MathNet.Numerics.LinearAlgebra;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public class DampedLeastSquares : InverseKinematicsSolver
    {
        private float _lambda = 0.5f;
        public ref float Lambda => ref _lambda;

        public DampedLeastSquares(float precision, float stepSize, int maxTime) : base(precision, stepSize, maxTime) { }  // TODO: remove stepSize for Jacobian solvers

        public override (bool, float, VectorFloat, bool[]) Execute(Obstacle[] Obstacles, Manipulator agent, Vector3 goal, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = agent.Joints.Length - 1;

            VectorFloat initConfig = agent.q, dq;
            for (int j = 0; j < _maxTime; j++)
            {
                // get positional/orientational error
                var error = GetError(agent, goal, joint);  // TODO: check for oscillations (the error starts increasing) and break if they appear

                // get Jacobian, its transpose and an identity matrix
                var J = Jacobian.Create(agent, joint);
                var JT = J.Transpose();
                var I = Matrix<float>.Build.DenseIdentity(error.Count);

                // calculate the displacement
                dq = -JT * (J * JT + _lambda * _lambda * I).Solve(error);

                // update maipulator's configuration
                agent.q = agent.q.AddSubVector(dq);
            }

            // checking for collisions of the found configuration
            bool[] collisions = DetectCollisions(agent, Obstacles);
            var dist = agent.Joints[joint].Position.DistanceTo(goal);

            return (true, dist, agent.q - initConfig, collisions);
        }
    }
}
