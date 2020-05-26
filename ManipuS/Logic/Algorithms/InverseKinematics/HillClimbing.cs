using System;
using System.Numerics;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    class HillClimbing : InverseKinematicsSolver
    {
        protected static float _stepSizeDefault = 2;

        protected float _stepSize;
        public ref float StepSize => ref _stepSize;

        public HillClimbing(float threshold, int maxIterations, float stepSize) : base(threshold, maxIterations) 
        {
            _stepSize = stepSize;
        }

        public static HillClimbing Default()
        {
            return new HillClimbing(_thresholdDefault, 10 * _maxIterationsDefault, _stepSizeDefault);
        }

        public override (bool, int, float, VectorFloat) Execute(Manipulator agent, Vector3 goal, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = agent.Joints.Length - 1;

            // initial parameters
            VectorFloat qBest = agent.q;
            float dist = agent.Joints[joint].Position.DistanceTo(goal), init_dist = dist, k = 1;
            float minDist = float.PositiveInfinity;
            bool converged = false;

            VectorFloat dq = VectorFloat.Build.Dense(agent.Joints.Length);
            float range, stepNeg, stepPos;
            int iters = 0;
            while (iters++ < MaxIterations)
            {
                for (int i = 0; i < joint; i++)
                {
                    // checking GC constraints
                    range = agent.Joints[i].CoordinateRange.X - agent.q[i] * 180 / (float)Math.PI;
                    stepNeg = range <= -_stepSize ? -_stepSize : range;

                    range = agent.Joints[i].CoordinateRange.Y - agent.q[i] * 180 / (float)Math.PI;
                    stepPos = range >= _stepSize ? _stepSize : range;

                    // generating random GCs' offset
                    dq[i] = (float)((stepNeg + RandomThreadStatic.NextDouble() * (stepPos - stepNeg)) * Math.PI / 180);
                    dq[i] *= k;
                }

                // retrieving score of the new configuration
                agent.q = qBest + dq;
                float distNew = agent.Joints[joint].Position.DistanceTo(goal);

                if (distNew < dist)
                {
                    // updating agent's configuration if it's better than the previos one
                    qBest = agent.q;
                    minDist = dist = distNew;
                    k = dist / init_dist;
                }

                if (dist < _threshold)
                {
                    // the algorithm has converged
                    converged = true;
                    break;
                }
            }

            return (converged, iters - 1, minDist, qBest);
        }
    }
}
