using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Logic
{
    /*public class Chromosome<T>
    {
        public int ParamNum;
        public T[] Genes;

        public Chromosome(int param_num)
        {
            ParamNum = param_num;
            Genes = new T[ParamNum];
        }

        public Chromosome(T[] genes)
        {
            ParamNum = genes.Length;
            Genes = genes;
        }

        public T GetParam(int num)
        {
            return Genes[num];
        }

        public T[] GetParamAll()
        {
            return Genes;
        }

        public S[] GetParamAll<S>(Func<T, S> func)
        {
            S[] _params = new S[ParamNum];
            for (int i = 0; i < ParamNum; i++)
                _params[i] = func(GetParam(i));

            return _params;
        }

        public string StrRep()
        {
            return string.Join("", Genes);
        }
    }*/

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

            // represent manipulator as a sequence of actuators
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

            // check collisions for all actuators
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
        //public Func<double, double> Decode;

        public HillClimbing(Obstacle[] obstacles, int paramNum, double precision, double stepSize, int maxTime) : base(obstacles, paramNum, precision, stepSize, maxTime) { }

        public Tuple<bool, double, double[], bool[]> Execute(Manipulator agent, Point goal)
        {
            double range = 0, step_neg = 0, step_pos = 0;
            for (int i = 0; i < ParamNum; i++)
            {
                range = agent.q_ranges[i, 0] - agent.q[i] * 180 / Math.PI;
                step_neg = range <= -StepSize ? -StepSize : range;

                range = agent.q_ranges[i, 1] - agent.q[i] * 180 / Math.PI;
                step_pos = range >= StepSize ? StepSize : range;
            }
            
            double[] qBest = Misc.CopyArray(agent.q);
            double dist = agent.DistanceTo(goal), init_dist = dist, scale = 1;
            double Min = double.PositiveInfinity;
            bool Converged = false;

            Stopwatch sw = new Stopwatch();
            sw.Start();
            double[] dq = new double[ParamNum];
            while (Time++ < MaxTime)
            {
                /*Chromosome<double> ch = new Chromosome<double>(ParamNum);
                for (int i = 0; i < ParamNum; i++)
                {
                    ch.Genes[i] = step_neg + Rng.NextDouble() * (step_pos - step_neg);
                    ch.Genes[i] *= coeff;
                }

                double[] dq = ch.GetParamAll(Decode);*/
                for (int i = 0; i < ParamNum; i++)
                {
                    dq[i] = (step_neg + Rng.NextDouble() * (step_pos - step_neg)) * Math.PI / 180;
                    dq[i] *= scale;
                }

                agent.q = qBest.Zip(dq, (t, s) => { return t + s; }).ToArray();
                double dist_new = agent.DistanceTo(goal);
                if (dist_new < dist)
                {
                    qBest = Misc.CopyArray(agent.q);
                    Min = dist = dist_new;
                    scale = dist / init_dist;
                }

                if (dist < Precision)
                {
                    Converged = true;
                    break;
                }
            }
            sw.Stop();
            Console.WriteLine("Time: {0}; Real time: {1}", Time, sw.ElapsedTicks / 10);

            bool[] Collisions = DetectCollisions(agent);

            Time = 0;
            return new Tuple<bool, double, double[], bool[]>(Converged, Min, qBest, Collisions);
        }
    }
}