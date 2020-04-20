using System.Numerics;

namespace Logic.InverseKinematics
{
    class Jacobian : IKSolver
    {
        public Jacobian(float precision, float stepSize, int maxTime) : base(precision, stepSize, maxTime) { }

        public override (bool, float, Vector, bool[]) Execute(Obstacle[] Obstacles, Manipulator agent, Vector3 goal, int joint)
        {
            Vector initConfig = agent.q;
            float alpha = 0.1f;
            for (int j = 0; j < 4; j++)
            {
                Vector3 jointPos = agent.Joints[joint].Position;
                Vector3 error = goal - jointPos;
                Vector errorExt = new Vector(error.X, error.Y, error.Z, 0, 0, 0);

                Vector[] data = new Vector[joint + 1];
                for (int i = 0; i <= joint; i++)
                {
                    var elem = Vector3.Cross(agent.Joints[i].Axis, jointPos - agent.Joints[i].Position);
                    if (elem != Vector3.Zero)
                        elem = Vector3.Normalize(elem);
                    data[i] = new Vector(
                        elem.X, 
                        elem.Y, 
                        elem.Z, 
                        agent.Joints[i].Axis.X, 
                        agent.Joints[i].Axis.Y, 
                        agent.Joints[i].Axis.Z);
                }

                // get transpose of the Jacobian
                Matrix JT = new Matrix(data);
                Matrix J = JT.Transpose;
                Vector JJTe = J * JT * errorExt;
                float nom = Vector.Dot(errorExt, JJTe);
                float denom = Vector.Dot(JJTe, JJTe);
                if (denom != 0)
                    alpha = nom / denom;

                // calculate GC offsets
                Vector dq = JT * errorExt;
                //dq *= -0.1f;
                dq *= -alpha;

                if (joint < agent.Joints.Length - 1)
                    dq.Expand(agent.Joints.Length - joint);

                agent.q += dq;
            }

            // checking for collisions of the found configuration
            bool[] collisions = DetectCollisions(agent, Obstacles);
            var dist = agent.Joints[joint].Position.DistanceTo(goal);

            return (true, dist, agent.q - initConfig, collisions);
        }
    }
}
