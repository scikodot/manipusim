using System;
using System.Collections.Generic;
using System.Linq;

namespace Logic
{
    class Algorithm
    {
        public static Random Rng;
        public Obstacle[] Obstacles;
        public double Precision, StepSize;
        public int ParamNum, MaxTime, Time = 0;

        public Algorithm(Obstacle[] obstacles, int paramNum, double precision, double stepSize, int maxTime)
        {
            Rng = new Random();
            Obstacles = obstacles;
            ParamNum = paramNum;
            Precision = precision;
            StepSize = stepSize;
            MaxTime = maxTime;
        }

        public bool[] DetectCollisions(Manipulator manip)
        {
            List<Point> joints = manip.DKP.ToList();
            
            // removing duplicates
            for (int i = 0; i < joints.Count - 1; i++)
            {
                if (joints[i].DistanceTo(joints[i + 1]) == 0)
                    joints.RemoveAt(i + 1);
            }

            // representing manipulator as a sequence of actuators
            List<Point> actuators = new List<Point>();
            int actuatorsNum = 50;
            for (int i = 0; i < joints.Count - 1; i++)
            {                
                actuators.Add(joints[i]);
                for (int j = 0; j < actuatorsNum; j++)
                {
                    actuators.Add(new Point
                    (
                        joints[i].x + (j + 1) * (joints[i + 1].x - joints[i].x) / (actuatorsNum + 1),
                        joints[i].y + (j + 1) * (joints[i + 1].y - joints[i].y) / (actuatorsNum + 1),
                        joints[i].z + (j + 1) * (joints[i + 1].z - joints[i].z) / (actuatorsNum + 1)
                    ));
                }
            }
            actuators.Add(joints[joints.Count - 1]);

            // checking collisions for all actuators
            bool[] collisions = new bool[joints.Count - 1];
            foreach (var obst in Obstacles)
            {
                for (int i = 0; i < actuators.Count; i++)
                {
                    if (obst.Contains(actuators[i]))
                    {
                        collisions[(i - 1) / (actuatorsNum + 1)] = true;
                        goto CollisionDetected;
                    }
                }
            }
            CollisionDetected:

            return collisions;
        }
    }

    class HillClimbing : Algorithm
    {
        public HillClimbing(Obstacle[] obstacles, int paramNum, double precision, double stepSize, int maxTime) : base(obstacles, paramNum, precision, stepSize, maxTime) { }

        public Tuple<bool, double, double[], bool[]> Execute(Manipulator agent, Point goal)
        {
            // initial parameters
            double[] qBest = Misc.CopyArray(agent.q);
            double dist = agent.DistanceTo(goal), init_dist = dist, k = 1;
            double min_dist = double.PositiveInfinity;
            bool Converged = false;
            
            double[] dq = new double[ParamNum];
            double range = 0, step_neg = 0, step_pos = 0;
            while (Time++ < MaxTime)
            {
                for (int i = 0; i < ParamNum; i++)
                {
                    // checking GC constraints
                    range = agent.q_ranges[i, 0] - agent.q[i] * 180 / Math.PI;
                    step_neg = range <= -StepSize ? -StepSize : range;

                    range = agent.q_ranges[i, 1] - agent.q[i] * 180 / Math.PI;
                    step_pos = range >= StepSize ? StepSize : range;

                    // generating random GCs' offset
                    dq[i] = (step_neg + Rng.NextDouble() * (step_pos - step_neg)) * Math.PI / 180;
                    dq[i] *= k;
                }

                // retrieving score of the new configuration
                agent.q = qBest.Zip(dq, (t, s) => { return t + s; }).ToArray();
                double dist_new = agent.DistanceTo(goal);

                if (dist_new < dist)
                {
                    // updating agent's configuration if it's better than the previos one
                    qBest = Misc.CopyArray(agent.q);
                    min_dist = dist = dist_new;
                    k = dist / init_dist;
                }

                if (dist < Precision)
                {
                    // the algorithm has converged
                    Converged = true;
                    break;
                }
            }

            // checking for collisions of the found configuration if the algorithm has converged
            bool[] Collisions = new bool[agent.q.Length - 1];
            if (Converged)
                Collisions = DetectCollisions(agent);

            // resetting timer
            Time = 0;

            return new Tuple<bool, double, double[], bool[]>(Converged, min_dist, qBest, Collisions);
        }
    }
}