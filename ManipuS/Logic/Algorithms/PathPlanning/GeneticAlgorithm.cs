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
    }

    static partial class PathPlanner
    {
        public static float[][] GeneticAlgorithm(Manipulator agent, Point goal, double precision, int paramNum, int genSize, double mutationProb, int maxTime, Func<double, double> decode)
        {
            var chs = new Chromosome<double>[genSize];
            var fit = new double[genSize];

            for (int i = 0; i < genSize; i++)
            {
                chs[i] = new Chromosome<double>(paramNum);
                for (int j = 0; j < paramNum; j++)
                    chs[i].Genes[j] = agent.Joints[i].qRanges[0] + (agent.Joints[i].qRanges[1] - agent.Joints[i].qRanges[0]) * Rng.NextDouble();
            }

            Chromosome<double> dominant = null;
            double min = double.PositiveInfinity;
            bool converged = false;
            Stopwatch sw = new Stopwatch();
            sw.Start();
            int time = 0;
            while (time++ < maxTime)
            {
                // fitting
                FitnessFunction(agent, goal, chs, fit, decode);
                Array.ConvertAll(fit, (t) => { return Math.Abs(t); });

                // qualification
                min = fit.Min();
                if (min < precision)
                {
                    dominant = chs[Array.IndexOf(fit, min)];
                    converged = true;
                    break;
                }

                // selection
                RouletteWheelSelection(chs, fit);

                // mutation
                Mutation(chs, mutationProb, t => t + -0.1 + 0.2 * Rng.NextDouble());

                // crossover
                Crossover(chs);
            }
            sw.Stop();
            Console.WriteLine("GA Time: {0}; Real time: {1}", time, sw.ElapsedTicks / 10);

            // if the dominant chromosome wasn't found, consider last result the best result
            if (dominant == null)
            {
                dominant = chs[Array.IndexOf(fit, min)];
            }

            //  chromosome
            double[] dq = dominant.GetParamAll(decode);

            //detect all collisions
            /*Manipulator AgentNext = new Manipulator(agent)
            {
                q = agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray()
            };
            bool[] Collisions = DetectCollisions(AgentNext);*/

            return null;
        }

        private static void FitnessFunction<T>(Manipulator agent, Point goal, Chromosome<T>[] Chs, double[] Fit, Func<T, double> decode)
        {
            Manipulator Contestant = new Manipulator(agent);
            for (int i = 0; i < Chs.Length; i++)
            {
                // extract parameters' values from chromosome
                double[] dq = Chs[i].GetParamAll(decode);
                Contestant.q = agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray();

                // apply fitness functions to the given chromosome
                Fit[i] = Contestant.DistanceTo(goal);
            }
        }

        private static void RouletteWheelSelection<T>(Chromosome<T>[] Chs, double[] Fit)
        {
            Chromosome<T>[] selection = new Chromosome<T>[Chs.Length];
            double total = 0;
            for (int i = 0; i < Chs.Length; i++)
            {
                total += 1 / Fit[i];
            }

            double[] sectors = new double[Chs.Length];
            for (int i = 0; i < Chs.Length; i++)
                sectors[i] = (1 / Fit[i]) / total * 100;

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
                for (int i = 0; i < chr.ParamNum; i++)
                {
                    if (Rng.NextDouble() < mutationProb)
                        chr.Genes[i] = mutate(chr.Genes[i]);
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
                int point = Rng.Next(0, Chs[0].ParamNum - 1);
                T[] Ch1 = new T[Chs[0].ParamNum], Ch2 = new T[Chs[0].ParamNum];
                for (int j = 0; j < Chs[0].ParamNum; j++)
                {
                    if (j <= point)
                    {
                        Ch1[j] = Chs[pairs[2 * i]].Genes[j];
                        Ch2[j] = Chs[pairs[2 * i + 1]].Genes[j];
                    }
                    else
                    {
                        Ch1[j] = Chs[pairs[2 * i + 1]].Genes[j];
                        Ch2[j] = Chs[pairs[2 * i]].Genes[j];
                    }
                }

                crossed[2 * i] = new Chromosome<T>(Ch1);
                crossed[2 * i + 1] = new Chromosome<T>(Ch2);
            }

            crossed.CopyTo(Chs, 0);
        }
    }
}