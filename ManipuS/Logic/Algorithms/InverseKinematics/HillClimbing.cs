using System;
using System.Linq;

namespace Logic.InverseKinematics
{
    class HillClimbing : IKSolver
    {
        public HillClimbing(Obstacle[] obstacles, float precision, float stepSize, int maxTime) : 
            base(obstacles, precision, stepSize, maxTime) { }

        public override (bool, float, float[], bool[]) Execute(Manipulator agent, Vector3 goal, int joint)
        {
            // initial parameters
            float[] qBest = Misc.CopyArray(agent.q);
            float dist = agent.DKP[joint].DistanceTo(goal), init_dist = dist, k = 1;
            float minDist = float.PositiveInfinity;
            bool Converged = false;

            float[] dq = new float[agent.Joints.Length];
            float range = 0, stepNeg = 0, stepPos = 0;
            int time = 0;
            Dispatcher.Timer.Start();
            while (time++ < MaxTime)
            {
                for (int i = 0; i < joint; i++)
                {
                    // checking GC constraints
                    range = agent.Joints[i].qRanges[0] - agent.q[i] * 180 / (float)Math.PI;
                    stepNeg = range <= -StepSize ? -StepSize : range;

                    range = agent.Joints[i].qRanges[1] - agent.q[i] * 180 / (float)Math.PI;
                    stepPos = range >= StepSize ? StepSize : range;

                    // generating random GCs' offset
                    dq[i] = (float)((stepNeg + Dispatcher.Rng.NextDouble() * (stepPos - stepNeg)) * Math.PI / 180);
                    dq[i] *= k;
                }

                // retrieving score of the new configuration
                agent.q = qBest.Zip(dq, (t, s) => { return t + s; }).ToArray();
                float distNew = agent.DKP[joint].DistanceTo(goal);

                if (distNew < dist)
                {
                    // updating agent's configuration if it's better than the previos one
                    qBest = Misc.CopyArray(agent.q);
                    minDist = dist = distNew;
                    k = dist / init_dist;
                }

                if (dist < Precision)
                {
                    // the algorithm has converged
                    Converged = true;
                    break;
                }
            }
            Dispatcher.Timer.Stop();
            Console.SetCursorPosition(0, 15);
            Console.WriteLine("IKP time: {0}; Real time: {1}", time, Dispatcher.Timer.ElapsedTicks / 10);
            Dispatcher.Timer.Reset();

            // checking for collisions of the found configuration if the algorithm has converged
            bool[] Collisions = new bool[agent.q.Length - 1];
            if (Converged)
                Collisions = DetectCollisions(agent, Obstacles);

            return (Converged, minDist, qBest, Collisions);
        }
    }
}
