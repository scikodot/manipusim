using System.Numerics;

using BulletSharp;

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

        public override InverseKinematicsResult Execute(Manipulator manipulator, Vector3 goal, int joint = -1)  // TODO: refactor
        {
            // use gripper if default joint
            if (joint == -1)
                joint = manipulator.Joints.Length - 1;

            VectorFloat configuration = manipulator.q, dq = VectorFloat.Build.Dense(manipulator.Joints.Length);
            var errorPos = goal - manipulator.Joints[joint].Position;
            var error = VectorFloat.Build.Dense(new float[] { errorPos.X, errorPos.Y, errorPos.Z, 0, 0, 0 });
            float distance = errorPos.Length();
            float scale = 1;

            int iterations = 0;
            while (distance > _threshold && iterations++ < _maxIterations)
            {
                float range, stepNeg, stepPos;
                for (int i = 0; i < joint; i++)
                {
                    // checking coordinate constraints
                    range = manipulator.Joints[i].CoordinateRange.X - configuration[i] * MathUtil.SIMD_DEGS_PER_RAD;
                    stepNeg = range <= -_stepSize ? -_stepSize : range;

                    range = manipulator.Joints[i].CoordinateRange.Y - configuration[i] * MathUtil.SIMD_DEGS_PER_RAD;
                    stepPos = range >= _stepSize ? _stepSize : range;

                    // generating random coordinate offset
                    dq[i] = (float)((stepNeg + RandomThreadStatic.NextDouble() * (stepPos - stepNeg)) * MathUtil.SIMD_RADS_PER_DEG);
                    dq[i] *= scale;
                }

                // get the new configuration
                var configurationNew = configuration.AddSubVector(dq);
                var fkRes = manipulator.ForwardKinematics(configurationNew);

                float distanceNew = fkRes.JointPositions[joint].DistanceTo(goal);
                if (distanceNew < distance)
                {
                    // update configuration, distance and scale
                    configuration = configurationNew;
                    errorPos = goal - fkRes.JointPositions[joint];
                    error = VectorFloat.Build.Dense(new float[] { errorPos.X, errorPos.Y, errorPos.Z, 0, 0, 0 });

                    distance = distanceNew;
                    scale *= distanceNew / distance;
                }
            }

            return new InverseKinematicsResult
            {
                Converged = iterations <= _maxIterations,
                Iterations = iterations - 1,
                Configuration = configuration,
                Error = error
            };
        }
    }
}
