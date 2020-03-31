using System;
using System.Linq;

namespace Logic.InverseKinematics
{
    class Jacobian : IKSolver
    {
        public Jacobian(Obstacle[] obstacles, float precision, float stepSize, int maxTime) : 
            base(obstacles, precision, stepSize, maxTime) { }

        public override (bool, float, float[], bool[]) Execute(Manipulator agent, Vector3 goal, int joint)
        {
            Vector offset = new Vector(agent.Joints.Length);
            for (int j = 0; j < 10; j++)
            {
                var grip = agent.DKP[joint];
                Vector3 gPos = new Vector3(grip.X, grip.Y, grip.Z);
                Vector3 tPos = new Vector3(goal.X, goal.Y, goal.Z);
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

                for (int i = 0; i < joint; i++)  // TODO: all the manipulator structure logic should be incapsulated (elsewhere)
                {
                    model *= Matrix4.CreateRotationY(dh[i].theta);
                    model *= Matrix4.CreateTranslation(dh[i].d * Vector3.UnitY);
                    model *= Matrix4.CreateRotationX(dh[i].alpha);
                    model *= Matrix4.CreateTranslation(dh[i].r * Vector3.UnitX);

                    if (i < joint - 1)
                    {
                        jAxes[i + 1] = model.Rotation * jAxes[0];
                        jPos[i + 1] = model.Translation;  // TODO: is it safe?
                    }
                }

                Vector[] data = new Vector[joint];
                for (int i = 0; i < joint; i++)
                {
                    var elem = Vector3.Cross(jAxes[i], gPos - jPos[i]);
                    data[i] = new Vector(elem.X, elem.Y, elem.Z, jAxes[i].X, jAxes[i].Y, jAxes[i].Z);
                }

                // get transpose of the Jacobian
                Matrix Jtr = new Matrix(data);

                // calculate GC offsets
                Vector dq = Jtr * new Vector(new float[6] { error.X, error.Y, error.Z, 0, 0, 0 });
                dq *= -0.1f;
                if (joint < agent.Joints.Length)
                    dq.Expand(agent.Joints.Length - joint);

                // checking for collisions of the found configuration if the algorithm has converged
                agent.q = agent.q.Zip(dq.Components, (t, s) => t + s).ToArray();
                bool[] collisions = new bool[agent.q.Length - 1];
                collisions = DetectCollisions(agent, Obstacles);

                offset += dq;
                var dist = agent.GripperPos.DistanceTo(goal);

                if (j == 3)
                    return (true, dist, offset.Components, collisions);
            }

            return default;
        }
    }
}
