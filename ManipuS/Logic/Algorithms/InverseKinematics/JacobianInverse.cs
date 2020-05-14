﻿using System.Numerics;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public class JacobianInverse : InverseKinematicsSolver
    {
        public JacobianInverse(float precision, float stepSize, int maxTime) : base(precision, stepSize, maxTime) { }

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

                // get Jacobian and its Moore-Penrose inverse, aka pseudoinverse
                var J = Jacobian.Create(agent, joint);
                var JP = J.PseudoInverse();

                // calculate the displacement
                dq = -JP * error;  // TODO: check why minus should always be presented for dq in these methods!

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