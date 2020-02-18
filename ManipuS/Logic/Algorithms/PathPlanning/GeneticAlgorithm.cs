using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Logic
{
    internal class Chromosome<T>
    {
        public int ParamNum;
        public int PointsNum;
        public T[] Genes;

        public Chromosome(int pointsNum, int paramNum)
        {
            PointsNum = pointsNum;
            ParamNum = paramNum;
            Genes = new T[PointsNum * ParamNum];
        }

        /*public Chromosome(T[] genes)
        {
            ParamNum = genes.Length;
            Genes = genes;
        }*/

        public T GetParam(int num)
        {
            return Genes[num];
        }

        public T[] GetParamSpecific(int point)
        {
            T[] _params = new T[ParamNum];
            for (int i = 0; i < ParamNum; i++)
                _params[i] = GetParam(i + point * ParamNum);

            return _params;
        }

        public S[] GetParamSpecific<S>(int point, Func<T, S> func)
        {
            S[] _params = new S[ParamNum];
            for (int i = 0; i < ParamNum; i++)
                _params[i] = func(GetParam(i + point * ParamNum));

            return _params;
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
    }

    static partial class PathPlanner
    {
        public static Point[] GeneticAlgorithm(Manipulator agent, Point goal, double precision, int paramNum, int genSize, double mutationProb, int maxTime, Func<double, double> decode)
        {
            var chs = new Chromosome<double>[genSize];
            var fit = new double[genSize];

            int pointsNum = 50;
            for (int i = 0; i < genSize; i++)
            {
                chs[i] = new Chromosome<double>(pointsNum, paramNum);
                for (int j = 0; j < pointsNum; j++)
                {
                    for (int k = 0; k < paramNum; k++)
                        chs[i].Genes[k + j * paramNum] = -1 + 2 * Rng.NextDouble();  //agent.Joints[k].qRanges[0] + (agent.Joints[k].qRanges[1] - agent.Joints[k].qRanges[0]) * Rng.NextDouble();
                }
            }

            Chromosome<double> dominant = null;
            double max = double.PositiveInfinity;
            bool converged = false;
            int time = 0;
            Stopwatch sw = new Stopwatch();
            sw.Start();
            while (time++ < maxTime)
            {
                if (time % 100 == 0)
                {
                    
                }

                // fitting
                FitnessFunction(agent, goal, chs, fit, decode);
                //Array.ConvertAll(fit, (t) => { return Math.Abs(t); });

                // qualification
                max = fit.Max();
                if (max > precision)
                {
                    dominant = chs[Array.IndexOf(fit, max)];
                    converged = true;
                    break;
                }

                // selection
                RouletteWheelSelection(chs, fit);

                // mutation
                Mutation(chs, mutationProb, t => t + -0.2 + 0.4 * Rng.NextDouble());

                // crossover
                Crossover(chs);
            }
            sw.Stop();
            Console.WriteLine("GA Time: {0}; Real time: {1}", time, sw.ElapsedTicks / 10);

            // if the dominant chromosome wasn't found, consider last result the best result
            if (dominant == null)
            {
                dominant = chs[Array.IndexOf(fit, max)];
            }

            // chromosome
            var points = new Point[chs[0].PointsNum];
            Manipulator AgentNext = new Manipulator(agent);
            for (int i = 0; i < chs[0].PointsNum; i++)
            {
                double[] dq = dominant.GetParamSpecific(i, decode);
                AgentNext.q = AgentNext.q.Zip(dq, (t, s) => { return t + s; }).ToArray();
                points[i] = AgentNext.GripperPos;
            }

            //detect all collisions
            /*Manipulator AgentNext = new Manipulator(agent)
            {
                q = agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray()
            };
            bool[] Collisions = DetectCollisions(AgentNext);*/

            return points;
        }

        private static void FitnessFunction<T>(Manipulator agent, Point goal, Chromosome<T>[] Chs, double[] Fit, Func<T, double> decode)
        {
            Manipulator Contestant = new Manipulator(agent);
            for (int i = 0; i < Chs.Length; i++)
            {
                Fit[i] = 0;

                Contestant.q = Misc.CopyArray(agent.q);
                Point contPrevPos = agent.GripperPos;

                // extract parameters' values from chromosome
                for (int j = 0; j < Chs[i].PointsNum; j++)
                {
                    double[] dq = Chs[i].GetParamSpecific(j, decode);
                    Contestant.q = Contestant.q.Zip(dq, (t, s) => { return t + s; }).ToArray();

                    // apply fitness functions to the given chromosome's point
                    double desDist = 0.1;
                    Point prevPos = contPrevPos;
                    Point currPos = Contestant.GripperPos;

                    double currDist = currPos.DistanceTo(prevPos);
                    if (currDist <= desDist)
                        Fit[i] += currDist / desDist;
                    else
                        Fit[i] += desDist / currDist;

                    contPrevPos = Contestant.GripperPos;
                }
            }
        }

        private static void RouletteWheelSelection<T>(Chromosome<T>[] Chs, double[] Fit)
        {
            Chromosome<T>[] selection = new Chromosome<T>[Chs.Length];
            /*double total = 0;
            for (int i = 0; i < Chs.Length; i++)
            {
                total += 1 / Fit[i];
            }

            double[] sectors = new double[Chs.Length];
            for (int i = 0; i < Chs.Length; i++)
                sectors[i] = (1 / Fit[i]) / total * 100;*/

            double total = Fit.Sum();

            double[] sectors = new double[Chs.Length];
            for (int i = 0; i < Chs.Length; i++)
                sectors[i] = Fit[i] / total * 100;

            // randomly select crossing chromosomes according to their weights
            for (int i = 0; i < Chs.Length; i++)
            {
                double num = Rng.NextDouble() * 100, seek = 0;

                for (int j = 0; j < Chs.Length; j++)
                {
                    seek += sectors[j];
                    if (num < seek)
                    {
                        selection[i] = Chs[j];
                        break;
                    }
                }
            }

            selection.CopyTo(Chs, 0);
        }

        private static void Mutation<T>(Chromosome<T>[] chs, double mutationProb, Func<T, T> mutate)
        {
            // randomly choose among chromosomes the mutating ones
            foreach (Chromosome<T> chr in chs)
            {
                if (Rng.NextDouble() < mutationProb)
                {
                    for (int i = 0; i < chr.PointsNum * chr.ParamNum; i++)
                    {
                        if (Rng.NextDouble() < mutationProb)
                            chr.Genes[i] = mutate(chr.Genes[i]);
                    }
                }
            }
        }

        private static void Crossover<T>(Chromosome<T>[] Chs)  // TODO: add different crossover types
        {
            Chromosome<T>[] crossed = new Chromosome<T>[Chs.Length];

            // form pairs
            int[] pairs = new int[Chs.Length];
            for (int i = 0; i < Chs.Length; i++)
                pairs[i] = i;

            // shuffle array of pairs to make them random
            int n = pairs.Length;
            while (n > 1)
            {
                int k = Rng.Next(n--);
                int t = pairs[n];
                pairs[n] = pairs[k];
                pairs[k] = t;
            }

            // crossover
            for (int i = 0; i < pairs.Length / 2; i++)
            {
                int point = Rng.Next(0, Chs[0].PointsNum - 1);
                Chromosome<T> Ch1 = new Chromosome<T>(Chs[0].PointsNum, Chs[0].ParamNum), 
                              Ch2 = new Chromosome<T>(Chs[0].PointsNum, Chs[0].ParamNum);
                for (int j = 0; j < Chs[0].PointsNum; j++)
                {
                    if (j <= point)
                    {
                        for (int k = 0; k < Chs[0].ParamNum; k++)
                        {
                            int index = k + j * Chs[0].ParamNum;
                            Ch1.Genes[index] = Chs[pairs[2 * i]].Genes[index];
                            Ch2.Genes[index] = Chs[pairs[2 * i + 1]].Genes[index];
                        }
                    }
                    else
                    {
                        for (int k = 0; k < Chs[0].ParamNum; k++)
                        {
                            int index = k + j * Chs[0].ParamNum;
                            Ch1.Genes[index] = Chs[pairs[2 * i + 1]].Genes[index];
                            Ch2.Genes[index] = Chs[pairs[2 * i]].Genes[index];
                        }
                    }
                }

                crossed[2 * i] = Ch1;
                crossed[2 * i + 1] = Ch2;
            }

            crossed.CopyTo(Chs, 0);
        }
    }
}