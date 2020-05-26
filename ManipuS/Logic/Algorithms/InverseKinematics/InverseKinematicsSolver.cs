using System;
using System.Numerics;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public enum InverseKinematicsSolverType
    {
        JacobianTranspose,
        JacobianInverse,
        DampedLeastSquares
    }

    public struct InverseKinematicsData  // TODO: perhaps "params" is better than "data"?
    {
        public int InverseKinematicsSolverID;
        public float StepSize;
        public float Precision;
        public int MaxTime;
    }

    public abstract class InverseKinematicsSolver
    {
        protected static float _thresholdDefault = 0.02f;
        protected static int _maxIterationsDefault = 50;

        public static string[] Types { get; } = Enum.GetNames(typeof(InverseKinematicsSolverType));

        public InverseKinematicsSolverType Type { get; private set; }

        protected float _threshold;
        public ref float Precision => ref _threshold;

        protected int _maxIterations;
        public ref int MaxIterations => ref _maxIterations;

        protected InverseKinematicsSolver(float precision, int maxTime)
        {
            _threshold = precision;
            _maxIterations = maxTime;

            Type = (InverseKinematicsSolverType)Enum.Parse(typeof(InverseKinematicsSolverType), GetType().Name);
        }

        protected VectorFloat GetError(Manipulator manipulator, Vector3 goal, int joint)
        {
            Vector3 error = goal - manipulator.Joints[joint].Position;
            return VectorFloat.Build.Dense(new float[]
            {
                error.X,
                error.Y,
                error.Z,
                0, 0, 0  // TODO: integrate orientation goal somehow?
            });
        }

        public abstract (bool, int, float, VectorFloat) Execute(Manipulator agent, Vector3 goal, int joint = -1);
    }
}
