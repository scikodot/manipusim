using System;
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace Logic.InverseKinematics
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public abstract class JacobianSolver : InverseKinematicsSolver
    {
        protected JacobianSolver(float precision, int maxIterations) : base(precision, maxIterations) { }

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
                if (elem != Vector3.Zero)
                    elem = Vector3.Normalize(elem);  // TODO: any use of normalization?

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

        public override InverseKinematicsResult Execute(Manipulator agent, Vector3 goal, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = agent.Joints.Length - 1;

            VectorFloat configuration = agent.q, dq;
            VectorFloat error;

            // TODO: check for oscillations (the error starts increasing) and break if they appear
            int iterations = 0;
            while (ErrorExceedsThreshold(agent, configuration, goal, joint, _threshold, 
                out ForwardKinematicsResult fkRes, out error) && iterations++ < _maxIterations)
            {
                // get generalized coordinates' offset
                dq = GetCoordinateOffset(CreateJacobian(fkRes, joint), error);

                // update configuration
                configuration = configuration.AddSubVector(dq);
            }

            return new InverseKinematicsResult
            {
                Converged = true/*iterations > _maxIterations*/,
                Iterations = iterations - 1,
                Configuration = configuration,
                Error = error
            };
        }

        protected abstract VectorFloat GetCoordinateOffset(Matrix<float> jacobian, VectorFloat error);
    }
}