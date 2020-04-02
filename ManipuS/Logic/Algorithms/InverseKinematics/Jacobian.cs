using System;
using System.Linq;

namespace Logic.InverseKinematics
{
    class Jacobian : IKSolver
    {
        public Jacobian(Obstacle[] obstacles, float precision, float stepSize, int maxTime) : 
            base(obstacles, precision, stepSize, maxTime) { }

        public override (bool, float, Vector, bool[]) Execute(Manipulator agent, Vector3 goal, int joint)
        {
            Vector offset = new Vector(agent.Joints.Length);
            for (int j = 0; j < 10; j++)
            {
                Vector3 grip = agent.DKP[joint];
                Vector3 error = goal - grip;

                Vector[] data = new Vector[joint];
                for (int i = 0; i < joint; i++)
                {
                    var elem = Vector3.Cross(agent.Joints[i].Axis, grip - agent.Joints[i].Position);
                    data[i] = new Vector(
                        elem.X, 
                        elem.Y, 
                        elem.Z, 
                        agent.Joints[i].Position.X, 
                        agent.Joints[i].Position.Y, 
                        agent.Joints[i].Position.Z);
                }

                // get transpose of the Jacobian
                Matrix Jtr = new Matrix(data);

                // calculate GC offsets
                Vector dq = Jtr * new Vector(error.X, error.Y, error.Z, 0, 0, 0);
                dq *= -0.1f;
                if (joint < agent.Joints.Length)
                    dq.Expand(agent.Joints.Length - joint);

                // checking for collisions of the found configuration if the algorithm has converged
                //agent.q = agent.q.Zip(dq.Components, (t, s) => t + s).ToArray();
                agent.q += dq;
                bool[] collisions = new bool[agent.q.Size - 1];
                collisions = DetectCollisions(agent, Obstacles);

                offset += dq;
                var dist = agent.GripperPos.DistanceTo(goal);

                if (j == 3)
                    return (true, dist, offset, collisions);
            }

            return default;
        }
    }
}
