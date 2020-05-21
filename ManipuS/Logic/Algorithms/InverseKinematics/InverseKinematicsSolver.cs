using Physics;
using System;
using System.Collections.Generic;
using System.Linq;
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
        public static string[] Types { get; } = Enum.GetNames(typeof(InverseKinematicsSolverType));

        protected float StepSize;  // TODO: move to where it's needed

        protected float _threshold;
        public ref float Precision => ref _threshold;

        protected int _maxTime;
        public ref int MaxTime => ref _maxTime;

        protected InverseKinematicsSolver(float precision, float stepSize, int maxTime)
        {
            _threshold = precision;
            StepSize = stepSize;
            MaxTime = maxTime;
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

        public abstract (bool, float, VectorFloat) Execute(Manipulator agent, Vector3 goal, int joint);
    }
}
