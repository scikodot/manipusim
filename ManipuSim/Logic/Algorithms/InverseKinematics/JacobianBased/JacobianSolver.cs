﻿using MathNet.Numerics.LinearAlgebra;

using BulletSharp.Math;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public abstract class JacobianSolver : InverseKinematicsSolver
    {
        protected JacobianSolver(float threshold, int maxIterations) : base(threshold, maxIterations) { }

        protected Matrix<float> CreateJacobian(ForwardKinematicsResult fkRes, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = fkRes.JointPositions.Length - 1;

            Vector3 jointPos = fkRes.JointPositions[joint];

            float[][] data = new float[joint + 1][];
            for (int i = 0; i <= joint; i++)
            {
                var elem = Vector3.Cross(fkRes.JointAxes[i], jointPos - fkRes.JointPositions[i]);
                //if (elem != Vector3.Zero)
                //    elem = Vector3.Normalize(elem);  // TODO: any use of normalization?

                data[i] = new float[]
                {
                        elem.X,
                        elem.Y,
                        elem.Z,
                        fkRes.JointAxes[i].X,
                        fkRes.JointAxes[i].Y,
                        fkRes.JointAxes[i].Z
                };
            }

            return Matrix<float>.Build.DenseOfColumnArrays(data);
        }

        public override InverseKinematicsResult Execute(Manipulator manipulator, Vector3 goal, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = manipulator.Joints.Length - 1;

            VectorFloat configuration = manipulator.Coordinates, dq;
            VectorFloat error;

            // TODO: check for oscillations (the error starts increasing) and break if they appear
            int iterations = 0;
            while (ErrorExceedsThreshold(manipulator, configuration, goal, joint, 
                out ForwardKinematicsResult fkRes, out error) && iterations < _maxIterations)
            {
                errorMod.Add(error.L2Norm());
                configs.Add(VectorFloat.Build.DenseOfVector(configuration));

                // get generalized coordinates' offset
                dq = GetCoordinateOffset(CreateJacobian(fkRes, joint), error);

                // update configuration
                configuration.AddSubVector(dq);

                //for (int i = 0; i < manipulator.Joints.Length; i++)
                //{
                //    manipulator.Joints[i].InitialCoordinate = configuration[i];
                //}

                iterations++;
            }

            return new InverseKinematicsResult
            {
                Converged = iterations < _maxIterations && !JointLimitsExceeded(manipulator, configuration),
                Iterations = iterations,
                Configuration = configuration,
                Error = error
            };
        }

        protected abstract VectorFloat GetCoordinateOffset(Matrix<float> jacobian, VectorFloat error);
    }
}