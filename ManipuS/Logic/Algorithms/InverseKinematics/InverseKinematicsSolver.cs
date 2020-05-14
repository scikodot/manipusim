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

        protected float _precision;
        public ref float Precision => ref _precision;

        protected int _maxTime;
        public ref int MaxTime => ref _maxTime;

        protected InverseKinematicsSolver(float precision, float stepSize, int maxTime)
        {
            _precision = precision;
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

        public bool[] DetectCollisions(Manipulator manip, Obstacle[] obstacles)
        {
            Vector3[] joints = manip.DKP;

            // representing manipulator as a sequence of actuators
            int actuatorsNum = 50;
            List<Vector3> actuators = new List<Vector3>();
            for (int i = 0; i < joints.Length - 1; i++)
            {
                actuators.Add(joints[i]);
                for (int j = 0; j < actuatorsNum; j++)
                {
                    actuators.Add(joints[i] + (j + 1) * (joints[i + 1] - joints[i]) / (actuatorsNum + 1));
                }
            }
            actuators.Add(joints[joints.Length - 1]);

            // checking collisions for all actuators
            bool[] collisions = new bool[joints.Length - 1];
            foreach (var obst in obstacles)
            {
                for (int i = 0; i < actuators.Count; i++)
                {
                    if (obst.Contains(actuators[i]))
                    {
                        collisions[(i - 1) / (actuatorsNum + 1)] = true;
                        goto CollisionDetected;
                    }
                }
            }
        CollisionDetected:

            return collisions;
        }

        public abstract (bool, float, MathNet.Numerics.LinearAlgebra.Vector<float>, bool[]) Execute(Obstacle[] Obstacles, Manipulator agent, Vector3 goal, int joint);
    }
}
