using System.Numerics;

using BulletSharp;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    class HillClimbing : InverseKinematicsSolver
    {
        protected static float _maxStepSizeDefault = 2;

        protected float _maxStepSize;
        public ref float MaxStepSize => ref _maxStepSize;

        public HillClimbing(float threshold, int maxIterations, float maxStepSize) : base(threshold, maxIterations)
        {
            _maxStepSize = maxStepSize;
        }

        public static HillClimbing Default()
        {
            return new HillClimbing(_thresholdDefault, 10 * _maxIterationsDefault, _maxStepSizeDefault);
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
            while (distance > _threshold && iterations < _maxIterations)
            {
                float range, stepNeg, stepPos;
                for (int i = 0; i < joint; i++)
                {
                    // checking coordinate constraints
                    range = manipulator.Joints[i].CoordinateRange.X - configuration[i] * MathUtil.SIMD_DEGS_PER_RAD;
                    stepNeg = range <= -_maxStepSize ? -_maxStepSize : range;

                    range = manipulator.Joints[i].CoordinateRange.Y - configuration[i] * MathUtil.SIMD_DEGS_PER_RAD;
                    stepPos = range >= _maxStepSize ? _maxStepSize : range;

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

                iterations++;
            }

            return new InverseKinematicsResult
            {
                Converged = iterations <= _maxIterations && !JointLimitsExceeded(manipulator, configuration),
                Iterations = iterations,
                Configuration = configuration,
                Error = error
            };
        }
    }
}
