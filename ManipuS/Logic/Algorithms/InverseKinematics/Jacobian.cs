using System;
using System.Linq;
using OpenTK;

namespace Logic.InverseKinematics
{
    class Jacobian : IKSolver
    {
        public Jacobian(Obstacle[] obstacles, double precision, double stepSize, int maxTime) : 
            base(obstacles, precision, stepSize, maxTime) { }

        public override (bool, double, double[], bool[]) Execute(Manipulator agent, Point goal, int joint)
        {
            VectorN offset = new VectorN(agent.Joints.Length);
            for (int j = 0; j < 10; j++)
            {
                var grip = agent.DKP[joint];
                Vector3 gPos = new Vector3((float)grip.x, (float)grip.y, (float)grip.z);
                Vector3 tPos = new Vector3((float)goal.x, (float)goal.y, (float)goal.z);
                Vector3 error = tPos - gPos;

                // get all joints parameters (positions/axes)
                var joints = agent.Joints;
                var dh = agent.DH;
                //var jointsCount = agent.Joints.Length;

                Matrix4 model = Matrix4.Identity;

                Vector3[] jAxes = new Vector3[joint];
                Vector3[] jPos = new Vector3[joint];

                jAxes[0] = Vector3.UnitY;
                jPos[0] = Vector3.Zero;

                Vector4 initAxis = Vector4.UnitY;
                for (int i = 0; i < joint; i++)  // TODO: all the manipulator structure logic should be incapsulated (elsewhere)
                {
                    model *= Matrix.RotateY((float)dh[i].theta(joints[i]));
                    model *= Matrix.Translate((float)dh[i].d * Vector3.UnitY);
                    model *= Matrix.RotateX((float)dh[i].alpha);
                    model *= Matrix.Translate((float)dh[i].r * Vector3.UnitX);

                    if (i < joint - 1)
                    {
                        jAxes[i + 1] = new Vector3(model * initAxis).Normalized();
                        jPos[i + 1] = new Vector3(model.M14, model.M24, model.M34);
                    }
                }

                // calculate the Jacobian
                double[,] data = new double[6, joint];
                for (int i = 0; i < joint; i++)
                {
                    var elem = Vector3.Cross(jAxes[i], gPos - jPos[i]);
                    data[0, i] = elem.X;
                    data[1, i] = elem.Y;
                    data[2, i] = elem.Z;
                    data[3, i] = jAxes[i].X;
                    data[4, i] = jAxes[i].Y;
                    data[5, i] = jAxes[i].Z;
                }

                Matrix J = new Matrix(data);

                // get transpose of the Jacobian
                Matrix Jtr = Matrix.Transpose(J);

                // calculate GC offsets
                VectorN dq = Jtr * new VectorN(new double[6] { error.X, error.Y, error.Z, 0, 0, 0 });
                dq *= -0.1;
                if (joint < agent.Joints.Length)
                    dq.Data = dq.Data.Concat(new double[agent.Joints.Length - joint]).ToArray();

                // checking for collisions of the found configuration if the algorithm has converged
                agent.q = agent.q.Zip(dq.Data, (t, s) => t + s).ToArray();
                bool[] collisions = new bool[agent.q.Length - 1];
                collisions = DetectCollisions(agent, Obstacles);

                offset += dq;
                var dist = agent.GripperPos.DistanceTo(goal);

                if (j == 9)
                    return (true, dist, offset.Data, collisions);
            }

            return default;
        }
    }
}
