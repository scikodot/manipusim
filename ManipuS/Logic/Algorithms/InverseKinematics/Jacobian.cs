using System;
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace Logic.InverseKinematics
{
    public static class Jacobian
    {
        public static Matrix<float> Create(Manipulator manipulator, int joint = -1)
        {
            // use gripper if default joint
            if (joint == -1)
                joint = manipulator.Joints.Length - 1;

            Vector3 jointPos = manipulator.Joints[joint].Position;

            float[][] data = new float[joint + 1][];
            for (int i = 0; i <= joint; i++)
            {
                var elem = Vector3.Cross(manipulator.Joints[i].Axis, jointPos - manipulator.Joints[i].Position);
                if (elem != Vector3.Zero)
                    elem = Vector3.Normalize(elem);  // TODO: any use of normalization?

                data[i] = new float[]
                {
                        elem.X,
                        elem.Y,
                        elem.Z,
                        manipulator.Joints[i].Axis.X,
                        manipulator.Joints[i].Axis.Y,
                        manipulator.Joints[i].Axis.Z
                };
            }

            return Matrix<float>.Build.DenseOfColumnArrays(data);
        }
    }
}