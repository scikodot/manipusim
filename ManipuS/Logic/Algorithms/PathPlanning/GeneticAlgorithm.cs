//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;

//namespace Logic.PathPlanning
//{
//    internal class Chromosome<T>
//    {
//        public int ParamNum;
//        public int PointsNum;
//        public T[][] Genes;

//        public Chromosome(int pointsNum, int paramNum)
//        {
//            PointsNum = pointsNum;
//            ParamNum = paramNum;
//            Genes = new T[PointsNum][];
//            for (int i = 0; i < PointsNum; i++)
//            {
//                Genes[i] = new T[ParamNum];
//            }
//        }

//        /*public Chromosome(T[] genes)
//        {
//            ParamNum = genes.Length;
//            Genes = genes;
//        }*/

//        public T GetParam(int point, int param)
//        {
//            return Genes[point][param];
//        }

//        public T[] GetGene(int point)
//        {
//            return Genes[point];
//        }

//        public S[] GetGene<S>(int point, Func<T, S> func)
//        {
//            S[] _params = new S[ParamNum];
//            for (int i = 0; i < ParamNum; i++)
//                _params[i] = func(GetParam(point, i));

//            return _params;
//        }
//    }

//    static partial class PathPlanner
//    {
//        public static (List<Point>, List<double[]>) GeneticAlgorithm(Manipulator agent, Obstacle[] obstacles, Point goal, double[][] initConfigs, 
//            double precision, int paramNum, int genSize, double crossoverProb, double mutationProb, int maxTime, 
//            OptimizationCriterion criteria, 
//            SelectionMode selectMode, 
//            CrossoverMode crossMode, 
//            Func<double, double> decode)
//        {
//            double[][] initSolution = new double[initConfigs.Length - 1][];
//            Array.Copy(initConfigs, 1, initSolution, 0, initConfigs.Length - 1);

//            var chs = new Chromosome<double>[genSize];
//            var fit = new double[genSize];

//            int pointsNum = initSolution.Length;
//            for (int i = 0; i < genSize; i++)
//            {
//                chs[i] = new Chromosome<double>(pointsNum, paramNum);
//                var param = Rng.Next(0, chs[i].ParamNum);
//                var amp = -10 + 20 * Rng.NextDouble();
//                for (int j = 0; j < pointsNum; j++)
//                {
//                    for (int k = 0; k < paramNum; k++)
//                    {
//                        chs[i].Genes[j][k] = initSolution[j][k] * 180 / Math.PI;
//                        if (k == param)
//                            chs[i].Genes[j][k] += amp;
//                    }
//                }

//                /*if (i == 0)
//                {
//                    Manipulator temp = new Manipulator(agent);
//                    List<Point> points = new List<Point>();
//                    for (int j = 0; j < chs[i].PointsNum; j++)
//                    {
//                        var config = chs[i].GetGene(j, decode);
//                        temp.q = Misc.CopyArray(config);
//                        points.Add(temp.GripperPos);
//                    }
//                    agent.points = points.ToArray();
//                }*/
//            }

//            Chromosome<double> dominant = null;
//            double max = 0;
//            bool converged = false;
//            int time = 0;
//            Stopwatch sw = new Stopwatch();
//            sw.Start();
//            while (time++ < maxTime)
//            {
//                if (time % 100 == 0)
//                {
                    
//                }

//                // fitting
//                FitnessFunction(agent, obstacles, goal, chs, fit, criteria, decode);
//                //Array.ConvertAll(fit, (t) => { return Math.Abs(t); });

//                // qualification
//                max = fit.Max();

//                dominant = chs[Array.IndexOf(fit, max)];
//                Manipulator temp = new Manipulator(agent);
//                List<Point> points = new List<Point>();
//                for (int j = 0; j < dominant.PointsNum; j++)
//                {
//                    var config = dominant.GetGene(j, decode);
//                    temp.q = Misc.CopyArray(config);
//                    points.Add(temp.GripperPos);
//                }
//                agent.points = points.ToArray();

//                Console.SetCursorPosition(0, 10);
//                Console.WriteLine($"Goal fit: {pointsNum * 100}");
//                Console.WriteLine($"Max  fit: {max}");
//                //if (max > precision * pointsNum * 100)
//                if (max >= pointsNum * 100)
//                {
//                    dominant = chs[Array.IndexOf(fit, max)];
//                    converged = true;
//                    break;
//                }

//                // selection
//                RouletteWheelSelection(chs, fit, selectMode, OptimizationMode.Maximum);

//                // mutation
//                Mutation(chs, mutationProb, t => t + -1 + 2 * Rng.NextDouble());

//                // crossover
//                Crossover(chs, fit, crossoverProb, crossMode);
//            }
//            sw.Stop();
//            Console.WriteLine("GA Time: {0}; Real time: {1}", time, sw.ElapsedTicks / 10);

//            // if the dominant chromosome wasn't found, consider last result the best result
//            if (dominant == null)
//            {
//                dominant = chs[Array.IndexOf(fit, max)];
//            }

//            // chromosome
//            List<Point> path = new List<Point>();
//            List<double[]> configs = new List<double[]>();
//            Manipulator AgentNext = new Manipulator(agent);
//            configs.Add(AgentNext.q);
//            path.Add(AgentNext.GripperPos);
//            for (int i = 0; i < pointsNum; i++)
//            {
//                var config = dominant.GetGene(i, decode);
//                AgentNext.q = Misc.CopyArray(config);
//                configs.Add(config);
//                path.Add(AgentNext.GripperPos);
//            }

//            // detect all collisions
//            /*Manipulator AgentNext = new Manipulator(agent)
//            {
//                q = agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray()
//            };
//            bool[] Collisions = DetectCollisions(AgentNext);*/

//            return (path, configs);
//        }

//        public enum OptimizationMode
//        {
//            Maximum,
//            Minimum
//        }

//        [Flags]
//        public enum OptimizationCriterion
//        {
//            GoalConvergence = 0,
//            CollisionFree = 1,
//            PathLength = 2,
//            PathSmoothness = 4
//        }

//        private static void FitnessFunction<T>(Manipulator agent, Obstacle[] obstacles, Point goal, Chromosome<T>[] chs, double[] fit, OptimizationCriterion criterion, Func<T, double> decode)
//        {
//            Manipulator Contestant = new Manipulator(agent);
//            //Vector goalVec = new Vector(goal);
//            for (int i = 0; i < chs.Length; i++)
//            {
//                fit[i] = 0;

//                Contestant.q = Misc.CopyArray(agent.q);
//                //Point contPrevPos = agent.GripperPos;

//                //double desDist = 0.1, currDist;
//                //Vector vec;
//                // extract parameters' values from chromosome
//                for (int j = 0; j < chs[i].PointsNum; j++)
//                {
//                    Contestant.q = Misc.CopyArray(chs[i].GetGene(j, decode));
//                    //Point prevPos = contPrevPos;
//                    Point currPos = Contestant.GripperPos;

//                    // TODO: add criteria for the first path point so that it stays close to the start

//                    // apply fitness functions to the given chromosome's point
//                    int critCount = 0;
//                    double pointWeight = 0;

//                    /*if (j == chs[i].PointsNum - 1)
//                    {
//                        double threshold = 0.1;
//                        double error = currPos.DistanceTo(goal);
//                        if (error <= threshold)
//                            pointWeight += 100;
//                        else
//                            pointWeight += 100 * threshold / error;
//                        critCount++;
//                    }*/

//                    if ((criterion & OptimizationCriterion.CollisionFree) == OptimizationCriterion.CollisionFree)
//                    {
//                        foreach (var obst in obstacles)
//                        {
//                            if (obst.Contains(currPos))
//                            {
//                                pointWeight += 100 * Math.Pow(currPos.DistanceTo(((Sphere)obst.Collider).Center) / ((Sphere)obst.Collider).Radius, 4);
//                            }
//                            else
//                            {
//                                pointWeight += 100;
//                            }
//                        }
//                        critCount++;
//                    }
//                    if ((criterion & OptimizationCriterion.PathLength) == OptimizationCriterion.PathLength)
//                    {

//                    }
//                    if ((criterion & OptimizationCriterion.PathSmoothness) == OptimizationCriterion.PathSmoothness)
//                    {

//                    }

//                    // take median of all criteria weights
//                    fit[i] += pointWeight / critCount;

//                    // TODO: apply goal convergence!
//                    /*double threshold = 0.1, scale = 1;
//                    if (j == chs[i].PointsNum - 1)
//                    {
//                        double error = currPos.DistanceTo(goal);
//                        if (error <= threshold)
//                            scale = 1;
//                        else
//                            scale = Math.Pow(threshold / error, 0.02);

//                        fit[i] *= scale;
//                    }*/

//                    /*currDist = currPos.DistanceTo(prevPos);
//                    if (currDist <= desDist)
//                        fit[i] += currDist / desDist;
//                    else
//                        fit[i] += desDist / currDist;*/
//                    //fit[i] += currPos.DistanceTo(prevPos);
//                    //vec = new Vector(prevPos, currPos);
//                    //fit[i] += Math.Abs(Math.Acos((vec.x * goalVec.x + vec.y * goalVec.y + vec.z * goalVec.z) / (vec.Length * goalVec.Length))) * 180 / Math.PI;

//                    //contPrevPos = Contestant.GripperPos;
//                }

//                /*currDist = goal.DistanceTo(contPrevPos);
//                if (currDist <= desDist)
//                    fit[i] += currDist / desDist;
//                else
//                    fit[i] += desDist / currDist;*/
//                //fit[i] += goal.DistanceTo(contPrevPos);
//                //vec = new Vector(contPrevPos, goal);
//                //fit[i] += Math.Abs(Math.Acos((vec.x * goalVec.x + vec.y * goalVec.y + vec.z * goalVec.z) / (vec.Length * goalVec.Length))) * 180 / Math.PI;
//            }
//        }

//        public enum SelectionMode
//        {
//            RouletteWheel,
//            NormalDistribution
//        }

//        private static void RouletteWheelSelection<T>(Chromosome<T>[] chs, double[] fit, SelectionMode selectMode, OptimizationMode optimizeMode)
//        {
//            Chromosome<T>[] selection = new Chromosome<T>[chs.Length];

//            // select chromosomes with the specified mode
//            switch (selectMode)
//            {
//                case SelectionMode.RouletteWheel:
//                    double[] sectors = new double[chs.Length];
//                    double total;
//                    switch (optimizeMode)
//                    {
//                        case OptimizationMode.Maximum:
//                            total = fit.Sum();
//                            for (int i = 0; i < chs.Length; i++)
//                                sectors[i] = fit[i] / total * 100;
//                            break;
//                        case OptimizationMode.Minimum:
//                            total = fit.Sum((x) => 1 / x);
//                            for (int i = 0; i < chs.Length; i++)
//                                sectors[i] = (1 / fit[i]) / total * 100;
//                            break;
//                    }

//                    // randomly select crossing chromosomes according to their weights
//                    for (int i = 0; i < chs.Length; i++)
//                    {
//                        double point = Rng.NextDouble() * 100, seek = 0;

//                        for (int j = 0; j < chs.Length; j++)
//                        {
//                            seek += sectors[j];
//                            if (point < seek)
//                            {
//                                selection[i] = chs[j];
//                                break;
//                            }
//                        }
//                    }
//                    break;
//                case SelectionMode.NormalDistribution:
//                    var fitList = fit
//                        .Select((x, i) => new KeyValuePair<int, double>(i, x))
//                        .OrderBy(x => x.Value)
//                        .ToList();
//                    var fitValues = fitList.Select(x => x.Value).ToList();
//                    var fitKeys = fitList.Select(x => x.Key).ToList();
//                    var min = fitValues[0];
//                    var max = fitValues[fitValues.Count - 1];
//                    double num;
//                    int index = 0;

//                    switch (optimizeMode)
//                    {
//                        case OptimizationMode.Maximum:
//                            for (int i = 0; i < chs.Length; i++)
//                            {
//                                num = Misc.BoxMullerTransform(Rng, max, (max - min) / 3);

//                                if (num <= min)
//                                    index = fitKeys[0];
//                                else if (num >= max)
//                                {
//                                    var diff = num - max;
//                                    num = max - diff;
//                                    index = fitKeys[fitValues.FindIndex(x => x > num)];
//                                }
//                                else
//                                    index = fitKeys[fitValues.FindIndex(x => x > num)];

//                                selection[i] = chs[index];
//                            }
//                            break;
//                        case OptimizationMode.Minimum:
//                            for (int i = 0; i < chs.Length; i++)
//                            {
//                                num = Misc.BoxMullerTransform(Rng, min, (max - min) / 3);
//                                if (num <= min)
//                                {
//                                    var diff = num - min;
//                                    num = min - diff;
//                                    index = fitKeys[fitValues.FindIndex(x => x < num)];
//                                }
//                                else if (num >= max)
//                                    index = fitKeys[fitValues.Count - 1];
//                                else
//                                    index = fitKeys[fitValues.FindIndex(x => x < num)];

//                                selection[i] = chs[index];
//                            }
//                            break;
//                    }
//                    break;
//            }

//            selection.CopyTo(chs, 0);
//        }

//        private static void Mutation(Chromosome<double>[] chs, double mutationProb, Func<double, double> mutate)
//        {
//            // randomly choose among chromosomes the mutating ones
//            foreach (var chr in chs)
//            {
//                if (Rng.NextDouble() < mutationProb)
//                {
//                    // slightly change randomly chosen parameter for all the points of the chromosome
//                    var param = Rng.Next(0, chr.ParamNum);
//                    var amp = mutate(0);
//                    var mu = Rng.Next(0, chr.PointsNum);
//                    double sigma = 10;
//                    double nd(int x) => amp * Math.Exp(-0.5 * Math.Pow((x - mu) / sigma, 2));
//                    for (int i = 0; i < chr.PointsNum; i++)
//                    {
//                        chr.Genes[i][param] += nd(i);
//                    }
//                }
//            }
//        }

//        public enum CrossoverMode
//        {
//            CrissCross,
//            WeightedMean
//        }

//        private static void Crossover(Chromosome<double>[] chs, double[] fit, double crossoverProb, CrossoverMode crossMode)  // TODO: add generic types; add crossover probability Pc
//        {
//            var crossed = new Chromosome<double>[chs.Length];

//            // form pairs
//            int[] pairs = new int[chs.Length];
//            for (int i = 0; i < chs.Length; i++)
//                pairs[i] = i;

//            // shuffle array of pairs to make them random
//            int n = pairs.Length;
//            while (n > 1)
//            {
//                int k = Rng.Next(n--);
//                int t = pairs[n];
//                pairs[n] = pairs[k];
//                pairs[k] = t;
//            }

//            // crossover with the specified mode according to probability
//            switch (crossMode)
//            {
//                case CrossoverMode.CrissCross:
//                    for (int i = 0; i < pairs.Length / 2; i++)
//                    {
//                        if (Rng.NextDouble() < crossoverProb)
//                        {
//                            int point = Rng.Next(0, chs[0].PointsNum - 1);
//                            Chromosome<double> ch1 = new Chromosome<double>(chs[0].PointsNum, chs[0].ParamNum),
//                                               ch2 = new Chromosome<double>(chs[0].PointsNum, chs[0].ParamNum);
//                            for (int j = 0; j < chs[0].PointsNum; j++)
//                            {
//                                if (j <= point)
//                                {
//                                    ch1.Genes[j] = chs[pairs[2 * i]].Genes[j];
//                                    ch2.Genes[j] = chs[pairs[2 * i + 1]].Genes[j];
//                                }
//                                else
//                                {
//                                    ch1.Genes[j] = chs[pairs[2 * i + 1]].Genes[j];
//                                    ch2.Genes[j] = chs[pairs[2 * i]].Genes[j];
//                                }
//                            }

//                            crossed[2 * i] = ch1;
//                            crossed[2 * i + 1] = ch2;
//                        }
//                        else
//                        {
//                            crossed[2 * i] = chs[pairs[2 * i]];
//                            crossed[2 * i + 1] = chs[pairs[2 * i + 1]];
//                        }
//                    }
//                    break;
//                case CrossoverMode.WeightedMean:
//                    double G1, G2, W1, W2;
//                    for (int i = 0; i < pairs.Length / 2; i++)
//                    {
//                        if (Rng.NextDouble() < crossoverProb)
//                        {
//                            int point = Rng.Next(0, chs[0].PointsNum - 1);
//                            Chromosome<double> ch1 = new Chromosome<double>(chs[0].PointsNum, chs[0].ParamNum),
//                                               ch2 = new Chromosome<double>(chs[0].PointsNum, chs[0].ParamNum);

//                            W1 = fit[pairs[2 * i]];
//                            W2 = fit[pairs[2 * i + 1]];
//                            for (int j = 0; j < chs[0].PointsNum; j++)
//                            {
//                                if (j <= point)
//                                {
//                                    ch1.Genes[j] = chs[pairs[2 * i]].Genes[j];
//                                    ch2.Genes[j] = chs[pairs[2 * i + 1]].Genes[j];
//                                }
//                                else
//                                {
//                                    for (int k = 0; k < chs[0].ParamNum; k++)
//                                    {
//                                        G1 = chs[pairs[2 * i]].Genes[j][k];
//                                        G2 = chs[pairs[2 * i + 1]].Genes[j][k];
//                                        ch1.Genes[j][k] = (G1 * W1 + G2 * W2) / (W1 + W2);
//                                        ch2.Genes[j][k] = (G1 * W2 + G2 * W1) / (W1 + W2);
//                                    }
//                                }
//                            }

//                            crossed[2 * i] = ch1;
//                            crossed[2 * i + 1] = ch2;
//                        }
//                        else
//                        {
//                            crossed[2 * i] = chs[pairs[2 * i]];
//                            crossed[2 * i + 1] = chs[pairs[2 * i + 1]];
//                        }
//                    }
//                    break;
//            }

//            crossed.CopyTo(chs, 0);
//        }
//    }
//}