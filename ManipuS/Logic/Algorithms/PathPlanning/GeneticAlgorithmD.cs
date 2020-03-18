//using System;
//using System.Collections.Generic;
//using System.Linq;
//using Logic.InverseKinematics;

//namespace Logic.PathPlanning
//{
//    internal class ChromosomeD  // TODO: the weight (fit) of the chromosome should be stored inside it
//    {
//        public int PointsNum;
//        public (Point, double[])[] Genes;

//        public ChromosomeD(int pointsNum)
//        {
//            PointsNum = pointsNum;
//            Genes = new (Point, double[])[PointsNum];
//        }

//        /*public Chromosome(T[] genes)
//        {
//            ParamNum = genes.Length;
//            Genes = genes;
//        }

//        public T GetParam(int point, int param)
//        {
//            return Genes[point][param];
//        }*/

//        public (Point, double[]) GetGene(int point)
//        {
//            return Genes[point];
//        }

//        /*public S[] GetGene<S>(int point, Func<T, S> func)
//        {
//            S[] _params = new S[ParamNum];
//            for (int i = 0; i < ParamNum; i++)
//                _params[i] = func(GetParam(point, i));

//            return _params;
//        }*/
//    }

//    static partial class PathPlanner
//    {
//        public static (List<Point>, List<double[]>) GeneticAlgorithmD(Manipulator agent, Obstacle[] obstacles, Point goal, IKSolver solver, 
//            (Point, double[])[] initSolution, double precision, int paramNum, int genSize, double crossoverProb, double mutationProb, int maxTime,
//            OptimizationCriterion criteria,
//            SelectionMode selectMode,
//            CrossoverMode crossMode,
//            Func<double, double> decode)
//        {
//            var chs = new ChromosomeD[genSize];
//            var fit = new double[genSize];

//            int pointsNum = initSolution.Length;
//            for (int i = 0; i < genSize; i++)
//            {
//                chs[i] = new ChromosomeD(pointsNum);

//                // define distortion amplitude as a normal distribution
//                var mu = Rng.Next(1, initSolution.Length - 1);
//                double sigma = 10;
//                var amp = -0.5 + 1 * Rng.NextDouble();
//                double nd(int t) => amp * Math.Exp(-0.5 * Math.Pow((t - mu) / sigma, 2));

//                // segment between prev and next points
//                var vec = new Vector(initSolution[mu - 1].Item1, initSolution[mu + 1].Item1);

//                // get distortion direction
//                var x = Rng.NextDouble();
//                var y = Rng.NextDouble();
//                var z = (vec.x * x + vec.y * y) / vec.z;
//                var dir = new Vector(x, y, z).Normalized;

//                // apply distortion to all points except start and end
//                chs[i].Genes[0].Item1 = initSolution[0].Item1;
//                for (int j = 1; j < pointsNum - 1; j++)
//                {
//                    chs[i].Genes[j].Item1 = initSolution[j].Item1 + dir * nd(j);
//                }
//                chs[i].Genes[initSolution.Length - 1].Item1 = initSolution[initSolution.Length - 1].Item1;

//                /*if (i == 0)
//                {
//                    agent.points = chs[i].Genes.Select(t => t.Item1).ToArray();
//                }*/
//            }

//            ChromosomeD dominant = null;
//            double max = 0;
//            bool converged = false;
//            int time = 0;
//            Dispatcher.Timer.Start();
//            while (time++ < maxTime)
//            {
//                if (time % 100 == 0)
//                {

//                }

//                // fitting
//                FitnessFunctionD(agent, obstacles, goal, solver, chs, fit, criteria, decode);

//                // qualification
//                max = fit.Max();

//                dominant = chs[Array.IndexOf(fit, max)];
//                agent.points = dominant.Genes.Select(t => t.Item1).ToArray();

//                Console.SetCursorPosition(0, 10);
//                Console.WriteLine($"Goal fit: {pointsNum * 100}");
//                Console.WriteLine($"Max  fit: {max}");
//                Console.WriteLine("GA Time: {0}; Real time: {1}", time, Dispatcher.Timer.ElapsedTicks / 10);
//                //if (max > precision * pointsNum * 100)
//                if (max >= pointsNum * 100)
//                {
//                    dominant = chs[Array.IndexOf(fit, max)];
//                    converged = true;
//                    break;
//                }

//                // selection
//                SelectionD(chs, fit, selectMode, OptimizationMode.Maximum);

//                // mutation
//                MutationD(chs, mutationProb, t => t + -0.1 + 0.2 * Rng.NextDouble());

//                // crossover
//                CrossoverD(chs, fit, crossoverProb, crossMode);
//            }
//            Dispatcher.Timer.Reset();

//            // if the dominant chromosome wasn't found, consider last result the best result
//            if (dominant == null)
//            {
//                dominant = chs[Array.IndexOf(fit, max)];
//            }

//            Manipulator contestant = new Manipulator(agent);
//            Dispatcher.Timer.Start();
//            for (int i = 0; i < dominant.PointsNum; i++)
//            {
//                if (i == 0)
//                {
//                    dominant.Genes[i].Item2 = Misc.CopyArray(agent.q);
//                }
//                else
//                {
//                    contestant.q = Misc.CopyArray(dominant.Genes[i - 1].Item2);
//                    var res = solver.Execute(contestant, dominant.Genes[i].Item1);

//                    // assign config to continue checking other points
//                    dominant.Genes[i].Item2 = Misc.CopyArray(contestant.q);
//                }
//            }
//            Dispatcher.Timer.Stop();
//            Console.WriteLine("Fit real time: {0}", Dispatcher.Timer.ElapsedTicks / 10);
//            Dispatcher.Timer.Reset();

//            // chromosome
//            List<Point> path = new List<Point>();
//            List<double[]> configs = new List<double[]>();
//            for (int i = 0; i < pointsNum; i++)
//            {
//                path.Add(dominant.Genes[i].Item1);
//                configs.Add(dominant.Genes[i].Item2);
//            }

//            // detect all collisions
//            /*Manipulator AgentNext = new Manipulator(agent)
//            {
//                q = agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray()
//            };
//            bool[] Collisions = DetectCollisions(AgentNext);*/

//            return (path, configs);
//        }

//        private static void FitnessFunctionD<T>(Manipulator agent, Obstacle[] obstacles, Point goal, IKSolver solver, 
//            ChromosomeD[] chs, double[] fit, OptimizationCriterion criterion, Func<T, double> decode)
//        {
//            Manipulator contestant = new Manipulator(agent);
//            for (int i = 0; i < chs.Length; i++)
//            {
//                fit[i] = 0;

//                contestant.q = Misc.CopyArray(agent.q);
//                // extract parameters' values from chromosome
//                for (int j = 0; j < chs[i].PointsNum; j++)
//                {
//                    Point currPos = chs[i].Genes[j].Item1;

//                    // apply fitness functions to the given chromosome's point
//                    int critCount = 0;
//                    double pointWeight = 0;

//                    /*if (j == 0)
//                    {
//                        chs[i].Genes[j].Item2 = Misc.CopyArray(agent.q);
//                        pointWeight += 100;
//                    }
//                    else
//                    {
//                        for (int k = 0; k < 1; k++)
//                        {
//                            contestant.q = Misc.CopyArray(chs[i].Genes[j - 1].Item2);
//                            var res = solver.Execute(contestant, chs[i].Genes[j].Item1);

//                            // assign config to continue checking other points
//                            chs[i].Genes[j].Item2 = Misc.CopyArray(contestant.q);

//                            if (res.Item1 && !res.Item4.Contains(true))
//                            {
//                                pointWeight += 100;
//                                break;
//                            }
//                        }
//                    }
//                    critCount++;*/

//                    if ((criterion & OptimizationCriterion.CollisionFree) == OptimizationCriterion.CollisionFree)
//                    {
//                        // TODO: instead of weighting, add "pulling-out", i.e. translating point along the common vector with the obst center; much better
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
//                }
//            }
//        }

//        private static void SelectionD(ChromosomeD[] chs, double[] fit, SelectionMode selectMode, OptimizationMode optimizeMode)
//        {
//            ChromosomeD[] selection = new ChromosomeD[chs.Length];

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
//                                    index = fitKeys[fitValues.FindIndex(x => x >= num)];
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
//                                    index = fitKeys[fitValues.FindIndex(x => x <= num)];
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

//        private static void MutationD(ChromosomeD[] chs, double mutationProb, Func<double, double> mutate)
//        {
//            // randomly choose among chromosomes the mutating ones
//            foreach (var chr in chs)
//            {
//                if (Rng.NextDouble() < mutationProb)
//                {
//                    // define distortion amplitude as a normal distribution
//                    var mu = Rng.Next(1, chr.PointsNum - 1);
//                    double sigma = 10;
//                    var amp = mutate(0);
//                    double nd(int t) => amp * Math.Exp(-0.5 * Math.Pow((t - mu) / sigma, 2));

//                    // segment between prev and next points
//                    var vec = new Vector(chr.Genes[mu - 1].Item1, chr.Genes[mu + 1].Item1);

//                    // get distortion direction
//                    var x = Rng.NextDouble();
//                    var y = Rng.NextDouble();
//                    var z = (vec.x * x + vec.y * y) / vec.z;
//                    var dir = new Vector(x, y, z).Normalized;

//                    // slightly change all the points of the chromosome except start and end
//                    for (int i = 1; i < chr.PointsNum - 1; i++)
//                    {
//                        chr.Genes[i].Item1 += dir * nd(i);

//                        // pull vertex to its neighbour to retain desired step distance
//                        vec = new Vector(chr.Genes[i].Item1, chr.Genes[i - 1].Item1);
//                        chr.Genes[i].Item1 += vec.Normalized * (vec.Length - 0.04);
//                    }
//                }
//            }
//        }

//        private static void CrossoverD(ChromosomeD[] chs, double[] fit, double crossoverProb, CrossoverMode crossMode)  // TODO: add generic types; add crossover probability Pc
//        {
//            var crossed = new ChromosomeD[chs.Length];

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
//                            ChromosomeD ch1 = new ChromosomeD(chs[0].PointsNum),
//                                        ch2 = new ChromosomeD(chs[0].PointsNum);
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
//                    (Point, double[]) G1, G2;
//                    double W1, W2;
//                    for (int i = 0; i < pairs.Length / 2; i++)
//                    {
//                        if (Rng.NextDouble() < crossoverProb)
//                        {
//                            int point = Rng.Next(0, chs[0].PointsNum - 1);
//                            ChromosomeD ch1 = new ChromosomeD(chs[0].PointsNum),
//                                        ch2 = new ChromosomeD(chs[0].PointsNum);

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
//                                    G1 = chs[pairs[2 * i]].Genes[j];
//                                    G2 = chs[pairs[2 * i + 1]].Genes[j];
//                                    ch1.Genes[j].Item1 = (G1.Item1 * W1 + G2.Item1 * W2) / (W1 + W2);
//                                    ch1.Genes[j].Item2 = G1.Item2;
//                                    ch2.Genes[j].Item1 = (G1.Item1 * W2 + G2.Item1 * W1) / (W1 + W2);
//                                    ch2.Genes[j].Item2 = G2.Item2;
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