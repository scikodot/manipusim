using System;
using System.Numerics;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public enum InverseKinematicsSolverType  // TODO: for scalability enum should be replaced with dictionary (?) to enable adding custom derived classes
    {
        JacobianTranspose,
        JacobianPseudoinverse,
        DampedLeastSquares,
        HillClimbing
    }

    public struct InverseKinematicsData  // TODO: perhaps "params" is better than "data"?
    {
        public int InverseKinematicsSolverID;
        public float StepSize;
        public float Precision;
        public int MaxTime;
    }

    public struct InverseKinematicsResult
    {
        public bool Converged;
        public int Iterations;
        public VectorFloat Configuration;
        public VectorFloat Error;
    }

    public abstract class InverseKinematicsSolver
    {
        protected static float _thresholdDefault = 0.02f;
        protected static int _maxIterationsDefault = 50;

        public static string[] Types { get; } = Enum.GetNames(typeof(InverseKinematicsSolverType));

        public InverseKinematicsSolverType Type { get; private set; }

        protected float _threshold;
        public ref float Threshold => ref _threshold;

        protected int _maxIterations;
        public ref int MaxIterations => ref _maxIterations;

        protected InverseKinematicsSolver(float threshold, int maxIterations)
        {
            _threshold = threshold;
            _maxIterations = maxIterations;

            Type = (InverseKinematicsSolverType)Enum.Parse(typeof(InverseKinematicsSolverType), GetType().Name);
        }

        protected bool ErrorExceedsThreshold(Manipulator manipulator, VectorFloat configuration, Vector3 goal, int joint, 
            out ForwardKinematicsResult fkRes, out VectorFloat error)
        {
            fkRes = manipulator.ForwardKinematics(configuration);
            var errorPos = goal - fkRes.JointPositions[joint];

            // TODO: integrate orientation goal somehow?
            error = VectorFloat.Build.Dense(new float[] { errorPos.X, errorPos.Y, errorPos.Z, 0, 0, 0 });

            return errorPos.Length() > _threshold;
        }

        public abstract InverseKinematicsResult Execute(Manipulator manipulator, Vector3 goal, int joint = -1);
    }
}
