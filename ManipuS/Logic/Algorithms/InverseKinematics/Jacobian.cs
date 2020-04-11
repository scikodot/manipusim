using System;
using System.Linq;

namespace Logic.InverseKinematics
{
    class Jacobian : IKSolver
    {
        public Jacobian(float precision, float stepSize, int maxTime) : base(precision, stepSize, maxTime) { }

        public override (bool, float, Vector, bool[]) Execute(Obstacle[] Obstacles, Manipulator agent, Vector3 goal, int joint)
        {
            Vector initConfig = agent.q;
            for (int j = 0; j < 2; j++)
            {
                Vector3 grip = agent.DKP[joint];
                Vector3 error = goal - grip;

                Vector[] data = new Vector[joint + 1];
                for (int i = 0; i <= joint; i++)
                {
                    var elem = Vector3.Cross(agent.Joints[i].Axis, grip - agent.Joints[i].Position).Normalized;
                    data[i] = new Vector(
                        elem.X, 
                        elem.Y, 
                        elem.Z, 
                        agent.Joints[i].Axis.X, 
                        agent.Joints[i].Axis.Y, 
                        agent.Joints[i].Axis.Z);
                }

                // get transpose of the Jacobian
                Matrix Jtr = new Matrix(data);

                // calculate GC offsets
                Vector dq = Jtr * new Vector(error.X, error.Y, error.Z, 0, 0, 0);
                dq *= -0.1f;

                if (joint < agent.Joints.Length - 1)
                    dq.Expand(agent.Joints.Length - joint);

                agent.q += dq;
            }

            // checking for collisions of the found configuration
            bool[] collisions = DetectCollisions(agent, Obstacles);
            var dist = agent.DKP[joint].DistanceTo(goal);

            return (true, dist, agent.q - initConfig, collisions);
        }
    }
}
