//using System;
//using System.Numerics;

//namespace Logic.InverseKinematics
//{
//    class HillClimbing : InverseKinematicsSolver
//    {
//        public HillClimbing(float precision, float stepSize, int maxTime) : base(precision, stepSize, maxTime) { }

//        public override (bool, float, Vector, bool[]) Execute(Obstacle[] Obstacles, Manipulator agent, Vector3 goal, int joint)
//        {
//            // initial parameters
//            Vector qBest = agent.q;
//            float dist = agent.Joints[joint].Position.DistanceTo(goal), init_dist = dist, k = 1;
//            float minDist = float.PositiveInfinity;
//            bool Converged = false;

//            Vector dq = new Vector(agent.Joints.Length);
//            float range, stepNeg, stepPos;
//            int time = 0;
//            while (time++ < MaxTime)
//            {
//                for (int i = 0; i < joint; i++)
//                {
//                    // checking GC constraints
//                    range = agent.Joints[i].CoordinateRange.X - agent.q[i] * 180 / (float)Math.PI;
//                    stepNeg = range <= -StepSize ? -StepSize : range;

//                    range = agent.Joints[i].CoordinateRange.Y - agent.q[i] * 180 / (float)Math.PI;
//                    stepPos = range >= StepSize ? StepSize : range;

//                    // generating random GCs' offset
//                    dq[i] = (float)((stepNeg + RandomThreadStatic.NextDouble() * (stepPos - stepNeg)) * Math.PI / 180);
//                    dq[i] *= k;
//                }

//                // retrieving score of the new configuration
//                agent.q = qBest + dq;
//                float distNew = agent.Joints[joint].Position.DistanceTo(goal);

//                if (distNew < dist)
//                {
//                    // updating agent's configuration if it's better than the previos one
//                    qBest = agent.q;
//                    minDist = dist = distNew;
//                    k = dist / init_dist;
//                }

//                if (dist < Precision)
//                {
//                    // the algorithm has converged
//                    Converged = true;
//                    break;
//                }
//            }

//            // checking for collisions of the found configuration if the algorithm has converged
//            bool[] Collisions = new bool[agent.q.Size - 1];
//            if (Converged)
//                Collisions = DetectCollisions(agent, Obstacles);

//            return (Converged, minDist, qBest, Collisions);
//        }
//    }
//}
