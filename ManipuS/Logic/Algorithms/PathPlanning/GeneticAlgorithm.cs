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
        public T[][] Genes;

        public Chromosome(int pointsNum, int paramNum)
        {
            PointsNum = pointsNum;
            ParamNum = paramNum;
            Genes = new T[PointsNum][];
            for (int i = 0; i < PointsNum; i++)
            {
                Genes[i] = new T[ParamNum];
            }
        }

        /*public Chromosome(T[] genes)
        {
            ParamNum = genes.Length;
            Genes = genes;
        }*/

        public T GetParam(int point, int param)
        {
            return Genes[point][param];
        }

        public T[] GetGene(int point)
        {
            return Genes[point];
        }

        public S[] GetGene<S>(int point, Func<T, S> func)
        {
            S[] _params = new S[ParamNum];
            for (int i = 0; i < ParamNum; i++)
                _params[i] = func(GetParam(point, i));

            return _params;
        }

        /*public T[] GetParamAll()
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
        }*/
    }

    static partial class PathPlanner
    {
        public static (List<Point>, List<double[]>) GeneticAlgorithm(Manipulator agent, Obstacle[] obstacles, Point goal, double[][] initConfigs, double precision, int paramNum, int genSize, double mutationProb, int maxTime, Func<double, double> decode)
        {
            double[][] initSolution = new double[initConfigs.Length - 1][];
            Array.Copy(initConfigs, 1, initSolution, 0, initConfigs.Length - 1);

            var chs = new Chromosome<double>[genSize];
            var fit = new double[genSize];

            int pointsNum = initSolution.Length;
            for (int i = 0; i < genSize; i++)
            {
                chs[i] = new Chromosome<double>(pointsNum, paramNum);
                var param = Rng.Next(0, paramNum);
                var offset = -10 + 20 * Rng.NextDouble();
                for (int j = 0; j < pointsNum; j++)
                {
                    for (int k = 0; k < paramNum; k++)
                    {
                        chs[i].Genes[j][k] = initSolution[j][k] * 180 / Math.PI;
                        if (k == param)
                            chs[i].Genes[j][k] += offset;
                    }
                    
                    /*for (int k = 0; k < paramNum; k++)
                        chs[i].Genes[j][k] = (initSolution[j][k] * 180 / Math.PI) + offset;  //agent.Joints[k].qRanges[0] + (agent.Joints[k].qRanges[1] - agent.Joints[k].qRanges[0]) * Rng.NextDouble();

                    /*if (j == 0)
                        chs[i].Genes[j] = chs[i].Genes[j].Zip(agent.q, (t, s) => t + s).ToArray();
                    else
                        chs[i].Genes[j] = chs[i].Genes[j].Zip(chs[i].Genes[j - 1], (t, s) => t + s).ToArray();*/

                    //chs[i].Genes[j] = Array.ConvertAll(Misc.CopyArray(configs[j]), (t) => t * 180 / Math.PI);
                }

                /*if (i == 0)
                {
                    Manipulator temp = new Manipulator(agent);
                    List<Point> points = new List<Point>();
                    for (int j = 0; j < chs[i].PointsNum; j++)
                    {
                        var config = chs[i].GetGene(j, decode);
                        temp.q = Misc.CopyArray(config);
                        points.Add(temp.GripperPos);
                    }
                    agent.points = points.ToArray();
                }*/
            }

            Chromosome<double> dominant = null;
            double max = 0;
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
                FitnessFunction(agent, obstacles, goal, chs, fit, decode);
                //Array.ConvertAll(fit, (t) => { return Math.Abs(t); });

                // qualification
                max = fit.Max();
                Console.SetCursorPosition(0, 10);
                Console.WriteLine($"Max fit: {max}");
                if (max > precision * pointsNum * 100)
                {
                    dominant = chs[Array.IndexOf(fit, max)];
                    converged = true;
                    break;
                }

                // selection
                RouletteWheelSelection(chs, fit);

                // mutation
                var mutationStrength = -5 + 10 * Rng.NextDouble();
                Mutation(chs, mutationProb, t => t + mutationStrength);

                // crossover
                Crossover(chs, fit);
            }
            sw.Stop();
            Console.WriteLine("GA Time: {0}; Real time: {1}", time, sw.ElapsedTicks / 10);

            // if the dominant chromosome wasn't found, consider last result the best result
            if (dominant == null)
            {
                dominant = chs[Array.IndexOf(fit, max)];
            }

            // chromosome
            List<Point> path = new List<Point>();
            List<double[]> configs = new List<double[]>();
            Manipulator AgentNext = new Manipulator(agent);
            configs.Add(AgentNext.q);
            path.Add(AgentNext.GripperPos);
            for (int i = 0; i < pointsNum; i++)
            {
                var config = dominant.GetGene(i, decode);
                AgentNext.q = Misc.CopyArray(config);
                configs.Add(config);
                path.Add(AgentNext.GripperPos);
            }

            // detect all collisions
            /*Manipulator AgentNext = new Manipulator(agent)
            {
                q = agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray()
            };
            bool[] Collisions = DetectCollisions(AgentNext);*/

            return (path, configs);
        }

        private static void FitnessFunction<T>(Manipulator agent, Obstacle[] obstacles, Point goal, Chromosome<T>[] chs, double[] fit, Func<T, double> decode)
        {
            Manipulator Contestant = new Manipulator(agent);
            //Vector goalVec = new Vector(goal);
            for (int i = 0; i < chs.Length; i++)
            {
                fit[i] = 0;

                Contestant.q = Misc.CopyArray(agent.q);
                //Point contPrevPos = agent.GripperPos;

                //double desDist = 0.1, currDist;
                //Vector vec;
                // extract parameters' values from chromosome
                for (int j = 0; j < chs[i].PointsNum; j++)
                {
                    Contestant.q = Misc.CopyArray(chs[i].GetGene(j, decode));

                    // apply fitness functions to the given chromosome's point
                    //Point prevPos = contPrevPos;
                    Point currPos = Contestant.GripperPos;

                    foreach (var obst in obstacles)
                    {
                        if (obst.Contains(currPos))
                        {
                            fit[i] += 100 * Math.Pow(currPos.DistanceTo(((Sphere)obst.Collider).Center) / ((Sphere)obst.Collider).Radius, 4);
                        }
                        else
                        {
                            fit[i] += 100;
                        }
                    }

                    /*currDist = currPos.DistanceTo(prevPos);
                    if (currDist <= desDist)
                        fit[i] += currDist / desDist;
                    else
                        fit[i] += desDist / currDist;*/
                    //fit[i] += currPos.DistanceTo(prevPos);
                    //vec = new Vector(prevPos, currPos);
                    //fit[i] += Math.Abs(Math.Acos((vec.x * goalVec.x + vec.y * goalVec.y + vec.z * goalVec.z) / (vec.Length * goalVec.Length))) * 180 / Math.PI;

                    //contPrevPos = Contestant.GripperPos;
                }

                /*currDist = goal.DistanceTo(contPrevPos);
                if (currDist <= desDist)
                    fit[i] += currDist / desDist;
                else
                    fit[i] += desDist / currDist;*/
                //fit[i] += goal.DistanceTo(contPrevPos);
                //vec = new Vector(contPrevPos, goal);
                //fit[i] += Math.Abs(Math.Acos((vec.x * goalVec.x + vec.y * goalVec.y + vec.z * goalVec.z) / (vec.Length * goalVec.Length))) * 180 / Math.PI;
            }
        }

        private static void RouletteWheelSelection<T>(Chromosome<T>[] chs, double[] fit)  // TODO: add different selection types, preferably by normal distribution (because for dense generations roulette wheel gives almost uniform distribution)
        {
            Chromosome<T>[] selection = new Chromosome<T>[chs.Length];

            /*double total = 0;
            for (int i = 0; i < chs.Length; i++)
            {
                total += 1 / fit[i];
            }

            double[] sectors = new double[chs.Length];
            for (int i = 0; i < chs.Length; i++)
                sectors[i] = (1 / fit[i]) / total * 100;

            double total = fit.Sum();

            double[] sectors = new double[chs.Length];
            for (int i = 0; i < chs.Length; i++)
                sectors[i] = fit[i] / total * 100;

            // randomly select crossing chromosomes according to their weights
            for (int i = 0; i < chs.Length; i++)
            {
                double num = Rng.NextDouble() * 100, seek = 0;

                for (int j = 0; j < chs.Length; j++)
                {
                    seek += sectors[j];
                    if (num < seek)
                    {
                        selection[i] = chs[j];
                        break;
                    }
                }
            }*/

            var fitList = fit
                .Select((x, i) => new KeyValuePair<int, double>(i, x))
                .OrderBy(x => x.Value)
                .ToList();
            var fitValues = fitList.Select(x => x.Value).ToList();
            var fitKeys = fitList.Select(x => x.Key).ToList();
            var min = fitValues[0];
            var max = fitValues[fitValues.Count - 1];
            double num;
            int index = 0;
            for (int i = 0; i < chs.Length; i++)
            {
                /*num = Misc.BoxMullerTransform(Rng, min, (max - min) / 3);
                if (num <= min)
                    index = fitKeys[0];
                else if (num >= max)
                    index = fitKeys[fitValues.Count - 1];
                else
                    index = fitKeys[fitValues.FindIndex(x => x < num)];*/

                num = Misc.BoxMullerTransform(Rng, max, (max - min) / 3);
                if (num <= min)
                    index = fitKeys[0];
                else if (num >= max)
                {
                    var diff = num - max;
                    num = max - diff;
                    index = fitKeys[fitValues.FindIndex(x => x > num)];
                }
                    //index = fitKeys[fitValues.Count - 1];
                else
                    index = fitKeys[fitValues.FindIndex(x => x > num)];

                selection[i] = chs[index];
            }

            selection.CopyTo(chs, 0);
        }

        private static void Mutation<T>(Chromosome<T>[] chs, double mutationProb, Func<T, T> mutate)
        {
            // randomly choose among chromosomes the mutating ones
            foreach (Chromosome<T> chr in chs)
            {
                if (Rng.NextDouble() < mutationProb)
                {
                    var param = Rng.Next(0, chr.ParamNum);
                    for (int i = 0; i < chr.PointsNum; i++)
                    {
                        chr.Genes[i][param] = mutate(chr.Genes[i][param]);
                    }
                }
            }
        }

        private static void Crossover(Chromosome<double>[] chs, double[] fit)  // TODO: add different crossover types
        {
            Chromosome<double>[] crossed = new Chromosome<double>[chs.Length];

            // form pairs
            int[] pairs = new int[chs.Length];
            for (int i = 0; i < chs.Length; i++)
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

            double G1, G2, W1, W2;
            // crossover
            for (int i = 0; i < pairs.Length / 2; i++)
            {
                int point = Rng.Next(0, chs[0].PointsNum - 1);
                Chromosome<double> ch1 = new Chromosome<double>(chs[0].PointsNum, chs[0].ParamNum), 
                                   ch2 = new Chromosome<double>(chs[0].PointsNum, chs[0].ParamNum);
                /*for (int j = 0; j < chs[0].PointsNum; j++)
                {
                    if (j <= point)
                    {
                        ch1.Genes[j] = chs[pairs[2 * i]].Genes[j];
                        ch2.Genes[j] = chs[pairs[2 * i + 1]].Genes[j];
                    }
                    else
                    {
                        ch1.Genes[j] = chs[pairs[2 * i + 1]].Genes[j];
                        ch2.Genes[j] = chs[pairs[2 * i]].Genes[j];
                    }
                }*/

                W1 = fit[pairs[2 * i]];
                W2 = fit[pairs[2 * i + 1]];
                for (int j = 0; j < chs[0].PointsNum; j++)
                {
                    if (j <= point)
                    {
                        ch1.Genes[j] = chs[pairs[2 * i]].Genes[j];
                        ch2.Genes[j] = chs[pairs[2 * i + 1]].Genes[j];
                    }
                    else
                    {
                        for (int k = 0; k < chs[0].ParamNum; k++)
                        {
                            G1 = chs[pairs[2 * i]].Genes[j][k];
                            G2 = chs[pairs[2 * i + 1]].Genes[j][k];
                            ch1.Genes[j][k] = (G1 * W1 + G2 * W2) / (W1 + W2);
                            ch2.Genes[j][k] = (G1 * W2 + G2 * W1) / (W1 + W2);
                        }
                    }
                }

                /*W1 = fit[pairs[2 * i]];
                W2 = fit[pairs[2 * i + 1]];
                for (int j = 0; j < chs[0].PointsNum; j++)
                {
                    for (int k = 0; k < chs[0].ParamNum; k++)
                    {
                        G1 = chs[pairs[2 * i]].Genes[j][k];
                        G2 = chs[pairs[2 * i + 1]].Genes[j][k];
                        ch1.Genes[j][k] = (G1 * W1 + G2 * W2) / (W1 + W2);
                        ch2.Genes[j][k] = (G1 * W2 + G2 * W1) / (W1 + W2);
                    }
                }*/

                crossed[2 * i] = ch1;
                crossed[2 * i + 1] = ch2;
            }

            crossed.CopyTo(chs, 0);
        }
    }
}