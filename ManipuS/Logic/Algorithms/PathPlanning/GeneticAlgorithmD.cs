//using System;
//using System.Collections.Generic;
//using System.Linq;
//using Logic.InverseKinematics;

//namespace Logic.PathPlanning
//{
//    internal class ChromosomeD  // TODO: the weight (fit) of the chromosome should be stored inside it
//    {
//        public int Vector3sNum;
//        public (Vector3, float[])[] Genes;

//        public ChromosomeD(int Vector3sNum)
//        {
//            Vector3sNum = Vector3sNum;
//            Genes = new (Vector3, float[])[Vector3sNum];
//        }

//        /*public Chromosome(T[] genes)
//        {
//            ParamNum = genes.Length;
//            Genes = genes;
//        }

//        public T GetParam(int Vector3, int param)
//        {
//            return Genes[Vector3][param];
//        }*/

//        public (Vector3, float[]) GetGene(int Vector3)
//        {
//            return Genes[Vector3];
//        }

//        /*public S[] GetGene<S>(int Vector3, Func<T, S> func)
//        {
//            S[] _params = new S[ParamNum];
//            for (int i = 0; i < ParamNum; i++)
//                _params[i] = func(GetParam(Vector3, i));

//            return _params;
//        }*/
//    }

//    static partial class PathPlanner
//    {
//        public static (List<Vector3>, List<float[]>) GeneticAlgorithmD(Manipulator agent, Obstacle[] obstacles, Vector3 goal, IKSolver solver, 
//            (Vector3, float[])[] initSolution, float precision, int paramNum, int genSize, float crossoverProb, float mutationProb, int maxTime,
//            OptimizationCriterion criteria,
//            SelectionMode selectMode,
//            CrossoverMode crossMode,
//            Func<float, float> decode)
//        {
//            var chs = new ChromosomeD[genSize];
//            var fit = new float[genSize];

//            int Vector3sNum = initSolution.Length;
//            for (int i = 0; i < genSize; i++)
//            {
//                chs[i] = new ChromosomeD(Vector3sNum);

//                // define distortion amplitude as a normal distribution
//                var mu = Rng.Next(1, initSolution.Length - 1);
//                float sigma = 10;
//                var amp = -0.5 + 1 * Rng.Nextfloat();
//                float nd(int t) => amp * Math.Exp(-0.5 * Math.Pow((t - mu) / sigma, 2));

//                // segment between prev and next Vector3s
//                var vec = new Vector(initSolution[mu - 1].Item1, initSolution[mu + 1].Item1);

//                // get distortion direction
//                var x = Rng.Nextfloat();
//                var y = Rng.Nextfloat();
//                var z = (vec.x * x + vec.y * y) / vec.z;
//                var dir = new Vector(x, y, z).Normalized;

//                // apply distortion to all Vector3s except start and end
//                chs[i].Genes[0].Item1 = initSolution[0].Item1;
//                for (int j = 1; j < Vector3sNum - 1; j++)
//                {
//                    chs[i].Genes[j].Item1 = initSolution[j].Item1 + dir * nd(j);
//                }
//                chs[i].Genes[initSolution.Length - 1].Item1 = initSolution[initSolution.Length - 1].Item1;

//                /*if (i == 0)
//                {
//                    agent.Vector3s = chs[i].Genes.Select(t => t.Item1).ToArray();
//                }*/
//            }

//            ChromosomeD dominant = null;
//            float max = 0;
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
//                agent.Vector3s = dominant.Genes.Select(t => t.Item1).ToArray();

//                Console.SetCursorPosition(0, 10);
//                Console.WriteLine($"Goal fit: {Vector3sNum * 100}");
//                Console.WriteLine($"Max  fit: {max}");
//                Console.WriteLine("GA Time: {0}; Real time: {1}", time, Dispatcher.Timer.ElapsedTicks / 10);
//                //if (max > precision * Vector3sNum * 100)
//                if (max >= Vector3sNum * 100)
//                {
//                    dominant = chs[Array.IndexOf(fit, max)];
//                    converged = true;
//                    break;
//                }

//                // selection
//                SelectionD(chs, fit, selectMode, OptimizationMode.Maximum);

//                // mutation
//                MutationD(chs, mutationProb, t => t + -0.1 + 0.2 * Rng.Nextfloat());

//                // crossover
//                CrossoverD(chs, fit, crossoverProb, crossMode);
//            }
//            Dispatcher.Timer.Reset();

//            // if the dominant chromosome wasn't found, consider last result the best result
//            if (dominant == null)  // TODO: at this Vector3 fit and chs arrays do not correlate, because chs was changed!
//            {
//                dominant = chs[Array.IndexOf(fit, max)];
//            }

//            Manipulator contestant = new Manipulator(agent);
//            Dispatcher.Timer.Start();
//            for (int i = 0; i < dominant.Vector3sNum; i++)
//            {
//                if (i == 0)
//                {
//                    dominant.Genes[i].Item2 = Misc.CopyArray(agent.q);
//                }
//                else
//                {
//                    contestant.q = Misc.CopyArray(dominant.Genes[i - 1].Item2);
//                    var res = solver.Execute(contestant, dominant.Genes[i].Item1);

//                    // assign config to continue checking other Vector3s
//                    dominant.Genes[i].Item2 = Misc.CopyArray(contestant.q);
//                }
//            }
//            Dispatcher.Timer.Stop();
//            Console.WriteLine("Fit real time: {0}", Dispatcher.Timer.ElapsedTicks / 10);
//            Dispatcher.Timer.Reset();

//            // chromosome
//            List<Vector3> path = new List<Vector3>();
//            List<float[]> configs = new List<float[]>();
//            for (int i = 0; i < Vector3sNum; i++)
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

//        private static void FitnessFunctionD<T>(Manipulator agent, Obstacle[] obstacles, Vector3 goal, IKSolver solver, 
//            ChromosomeD[] chs, float[] fit, OptimizationCriterion criterion, Func<T, float> decode)
//        {
//            Manipulator contestant = new Manipulator(agent);
//            for (int i = 0; i < chs.Length; i++)
//            {
//                fit[i] = 0;

//                contestant.q = Misc.CopyArray(agent.q);
//                // extract parameters' values from chromosome
//                for (int j = 0; j < chs[i].Vector3sNum; j++)
//                {
//                    Vector3 currPos = chs[i].Genes[j].Item1;

//                    // apply fitness functions to the given chromosome's Vector3
//                    int critCount = 0;
//                    float Vector3Weight = 0;

//                    /*if (j == 0)
//                    {
//                        chs[i].Genes[j].Item2 = Misc.CopyArray(agent.q);
//                        Vector3Weight += 100;
//                    }
//                    else
//                    {
//                        for (int k = 0; k < 1; k++)
//                        {
//                            contestant.q = Misc.CopyArray(chs[i].Genes[j - 1].Item2);
//                            var res = solver.Execute(contestant, chs[i].Genes[j].Item1);

//                            // assign config to continue checking other Vector3s
//                            chs[i].Genes[j].Item2 = Misc.CopyArray(contestant.q);

//                            if (res.Item1 && !res.Item4.Contains(true))
//                            {
//                                Vector3Weight += 100;
//                                break;
//                            }
//                        }
//                    }
//                    critCount++;*/

//                    if ((criterion & OptimizationCriterion.CollisionFree) == OptimizationCriterion.CollisionFree)
//                    {
//                        // TODO: instead of weighting, add "pulling-out", i.e. translating Vector3 along the common vector with the obst center; much better
//                        foreach (var obst in obstacles)
//                        {
//                            if (obst.Contains(currPos))
//                            {
//                                Vector3Weight += 100 * Math.Pow(currPos.DistanceTo(((Sphere)obst.Collider).Center) / ((Sphere)obst.Collider).Radius, 4);
//                            }
//                            else
//                            {
//                                Vector3Weight += 100;
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
//                    fit[i] += Vector3Weight / critCount;
//                }
//            }
//        }

//        private static void SelectionD(ChromosomeD[] chs, float[] fit, SelectionMode selectMode, OptimizationMode optimizeMode)
//        {
//            ChromosomeD[] selection = new ChromosomeD[chs.Length];

//            // select chromosomes with the specified mode
//            switch (selectMode)
//            {
//                case SelectionMode.RouletteWheel:
//                    float[] sectors = new float[chs.Length];
//                    float total;
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
//                        float Vector3 = Rng.Nextfloat() * 100, seek = 0;

//                        for (int j = 0; j < chs.Length; j++)
//                        {
//                            seek += sectors[j];
//                            if (Vector3 < seek)
//                            {
//                                selection[i] = chs[j];
//                                break;
//                            }
//                        }
//                    }
//                    break;
//                case SelectionMode.NormalDistribution:
//                    var fitList = fit
//                        .Select((x, i) => new KeyValuePair<int, float>(i, x))
//                        .OrderBy(x => x.Value)
//                        .ToList();
//                    var fitValues = fitList.Select(x => x.Value).ToList();
//                    var fitKeys = fitList.Select(x => x.Key).ToList();
//                    var min = fitValues[0];
//                    var max = fitValues[fitValues.Count - 1];
//                    float num;
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

//        private static void MutationD(ChromosomeD[] chs, float mutationProb, Func<float, float> mutate)
//        {
//            // randomly choose among chromosomes the mutating ones
//            foreach (var chr in chs)
//            {
//                if (Rng.Nextfloat() < mutationProb)
//                {
//                    // define distortion amplitude as a normal distribution
//                    var mu = Rng.Next(1, chr.Vector3sNum - 1);
//                    float sigma = 10;
//                    var amp = mutate(0);
//                    float nd(int t) => amp * Math.Exp(-0.5 * Math.Pow((t - mu) / sigma, 2));

//                    // segment between prev and next Vector3s
//                    var vec = new Vector(chr.Genes[mu - 1].Item1, chr.Genes[mu + 1].Item1);

//                    // get distortion direction
//                    var x = Rng.Nextfloat();
//                    var y = Rng.Nextfloat();
//                    var z = (vec.x * x + vec.y * y) / vec.z;
//                    var dir = new Vector(x, y, z).Normalized;

//                    // slightly change all the Vector3s of the chromosome except start and end
//                    for (int i = 1; i < chr.Vector3sNum - 1; i++)
//                    {
//                        chr.Genes[i].Item1 += dir * nd(i);

//                        // pull vertex to its neighbour to retain desired step distance
//                        vec = new Vector(chr.Genes[i].Item1, chr.Genes[i - 1].Item1);
//                        chr.Genes[i].Item1 += vec.Normalized * (vec.Length - 0.04);
//                    }
//                }
//            }
//        }

//        private static void CrossoverD(ChromosomeD[] chs, float[] fit, float crossoverProb, CrossoverMode crossMode)  // TODO: add generic types; add crossover probability Pc
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
//                        if (Rng.Nextfloat() < crossoverProb)
//                        {
//                            int Vector3 = Rng.Next(0, chs[0].Vector3sNum - 1);
//                            ChromosomeD ch1 = new ChromosomeD(chs[0].Vector3sNum),
//                                        ch2 = new ChromosomeD(chs[0].Vector3sNum);
//                            for (int j = 0; j < chs[0].Vector3sNum; j++)
//                            {
//                                if (j <= Vector3)
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
//                    (Vector3, float[]) G1, G2;
//                    float W1, W2;
//                    for (int i = 0; i < pairs.Length / 2; i++)
//                    {
//                        if (Rng.Nextfloat() < crossoverProb)
//                        {
//                            int Vector3 = Rng.Next(0, chs[0].Vector3sNum - 1);
//                            ChromosomeD ch1 = new ChromosomeD(chs[0].Vector3sNum),
//                                        ch2 = new ChromosomeD(chs[0].Vector3sNum);

//                            W1 = fit[pairs[2 * i]];
//                            W2 = fit[pairs[2 * i + 1]];
//                            for (int j = 0; j < chs[0].Vector3sNum; j++)
//                            {
//                                if (j <= Vector3)
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