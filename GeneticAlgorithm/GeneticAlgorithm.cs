using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Helper;
using WorkEnv;

namespace GeneticAlgorithm
{
    public class Chromosome<T>
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

    class Algorithm
    {
        public static Random Rng;

        //operation parameters
        public static Manipulator Agent;
        public static Obstacle[] Obstacles;

        public static void Initialize(Manipulator agent, Obstacle[] obstacles)
        {
            Rng = new Random();

            Agent = new Manipulator(agent);
            Obstacles = obstacles;
        }

        public static void RouletteWheelSelection<T>(Chromosome<T>[] Chs, double[] Fit)
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

            //randomly selecting crossing chromosomes according to their weights
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

        public static void Mutation<T>(Chromosome<T>[] Chs, double Pm, Func<T, T> mutate)
        {
            //randomly choosing among chromosomes which of them will mutate
            foreach (Chromosome<T> chr in Chs)
            {
                for (int i = 0; i < chr.ParamNum; i++)
                {
                    if (Rng.NextDouble() < Pm)
                        chr.Genes[i] = mutate(chr.Genes[i]);
                }
            }
        }

        public static void Crossover<T>(Chromosome<T>[] Chs)
        {
            Chromosome<T>[] crossed = new Chromosome<T>[Chs.Length];

            //forming pairs
            int[] pairs = new int[Chs.Length];
            for (int i = 0; i < Chs.Length; i++)
                pairs[i] = i;

            //shuffling array of pairs to make them random
            int n = pairs.Length;
            while (n > 1)
            {
                int k = Rng.Next(n--);
                int t = pairs[n];
                pairs[n] = pairs[k];
                pairs[k] = t;
            }

            //crossing
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

        public static bool[] DetectCollisions(Manipulator manip)
        {
            Point[] joints = manip.Joints;
            List<Point> manip_points = new List<Point>();

            bool[] collisions = new bool[Agent.q.Length];

            //check collision for each link
            int actuatorsNum = 50;
            double actuatorsOffset = 0.1;
            for (int i = 0; i < Agent.q.Length; i++)
            {
                /*//prepare actuators for current link
                Point[] pos = new Point[actuatorsNum];
                for (int j = 0; j < actuatorsNum; j++)
                {
                    pos[actuatorsNum - 1 - j] = new Point
                    (
                        joints[i].x - (j + 1) * Math.Abs(joints[i].x - (i == 0 ? 0 : joints[i - 1].x)) / (actuatorsNum + 1),
                        joints[i].y - (j + 1) * Math.Abs(joints[i].y - (i == 0 ? 0 : joints[i - 1].y)) / (actuatorsNum + 1)
                    );
                }

                Point[] actuators = new Point[actuatorsNum * 2];
                Vector aLink = new Vector
                (
                    joints[i].x - (i == 0 ? 0 : joints[i - 1].x),
                    joints[i].y - (i == 0 ? 0 : joints[i - 1].y)
                );
                for (int j = 0; j < actuatorsNum; j++)
                {
                    actuators[j] = pos[j] + aLink.Ortho.Normalized * actuatorsOffset;
                    actuators[j + actuatorsNum] = pos[j] - aLink.Ortho.Normalized * actuatorsOffset;
                }

                //check if any actuator detects collision
                foreach (var obst in Obstacles)
                {
                    for (int j = 0; j < actuators.Length; j++)
                    {
                        for (int k = 0; k < obst.Bounding.Length; k++)
                        {
                            if (obst.Contains(actuators[j]))
                            {
                                collisions[i] = true;
                                goto LoopBreak;
                            }
                        }
                    }
                }

                LoopBreak:
                continue;*/

                manip_points.Add(joints[i]);
                for (int j = 0; j < actuatorsNum; j++)
                {
                    manip_points.Add(new Point
                    (
                        joints[i].x + (j + 1) * (joints[i + 1].x - joints[i].x) / (actuatorsNum + 1),
                        joints[i].y + (j + 1) * (joints[i + 1].y - joints[i].y) / (actuatorsNum + 1)
                    ));
                }
            }
            manip_points.Add(joints[joints.Length - 1]);

            foreach (var obst in Obstacles)
            {
                for (int i = 0; i < manip_points.Count; i++)
                {
                    if (obst.Contains(manip_points[i]))
                    {
                        collisions[(i - 1) / (actuatorsNum + 1)] = true;
                        goto LoopBreak;
                    }
                }
            }
            LoopBreak:

            return collisions;
        }
    }

    class IKP : Algorithm
    {
        public static Point Goal;
        public static double Precision;
        public static int ParamNum, GenSize, Count, MaxTime, Time = 0, TotalRealTime = 0;
        public static double Pm;
        public static Chromosome<double>[] Chs;
        public static double[] Fit;
        public static Func<double, double> Decode;

        public IKP(double precision, int paramNum, int genSize, double p_m, int maxTime)
        {
            Precision = precision;
            ParamNum = paramNum;
            GenSize = genSize;
            Pm = p_m;
            MaxTime = maxTime;
            
            Decode = t => t * Math.PI / 180;
        }

        public Tuple<bool, double, double[], bool[]> Execute(Point goal)
        {
            Goal = goal;

            /*Chs = new Chromosome<double>[GenSize];
            Fit = new double[GenSize];

            for (int i = 0; i < GenSize; i++)
            {
                Chs[i] = new Chromosome<double>(ParamNum);
                for (int j = 0; j < ParamNum; j++)
                    Chs[i].Genes[j] = Agent.StepRanges[j, 0] + (Agent.StepRanges[j, 1] - Agent.StepRanges[j, 0]) * Rng.NextDouble();
            }

            Chromosome<double> dominant = null;
            double Min = double.PositiveInfinity;
            bool Converged = false;
            Stopwatch sw = new Stopwatch();
            sw.Start();
            while (Time++ < MaxTime)
            {
                //fitting
                FitnessFunction();
                Array.ConvertAll(Fit, (t) => { return Math.Abs(t); });

                //qualification
                Min = Fit.Min();
                if (Min < Precision)
                {
                    dominant = Chs[Array.IndexOf(Fit, Min)];
                    Converged = true;
                    break;
                }

                //selection
                RouletteWheelSelection(Chs, Fit);

                //mutation
                Mutation(Chs, Pm, t => t + -0.1 + 0.2 * Rng.NextDouble());

                //crossover
                Crossover(Chs);
            }
            sw.Stop();
            TotalRealTime += (int)sw.ElapsedTicks / 10;
            Console.WriteLine("IKP Time: {0}; Real time: {1}", Time, sw.ElapsedTicks / 10);

            //if dominant chromosome wasn't found, consider last result the best result
            if (dominant == null)
            {
                dominant = Chs[Array.IndexOf(Fit, Min)];
            }

            //decoding chromosome
            double[] dq = dominant.GetParamAll(Decode);

            //detect all collisions
            Manipulator AgentNext = new Manipulator(Agent)
            {
                q = Agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray()
            };
            bool[] Collisions = DetectCollisions(AgentNext);

            Time = 0;
            return new Tuple<bool, double, double[], bool[]>(Converged, Min, dq, Collisions);*/

            double range = 0;
            for (int i = 0; i < ParamNum; i++)
            {
                range = -120 * Math.PI / 180 - Agent.q[i];
                Agent.StepRanges[i, 0] = range <= -1 ? -1 : range;

                range = 120 * Math.PI / 180 - Agent.q[i];
                Agent.StepRanges[i, 1] = range >= 1 ? 1 : range;
            }

            Manipulator Contestant = new Manipulator(Agent);
            double[] q_init = new double[ParamNum];
            Array.Copy(Agent.q, q_init, ParamNum);
            double dist = Agent.DistanceTo(Goal), init_dist = dist, coeff = 1;
            double Min = double.PositiveInfinity;
            bool Converged = false;

            Stopwatch sw = new Stopwatch();
            sw.Start();
            while (Time++ < 500)
            {
                Chromosome<double> ch = new Chromosome<double>(ParamNum);
                for (int i = 0; i < ParamNum; i++)
                {
                    ch.Genes[i] = Agent.StepRanges[i, 0] + (Agent.StepRanges[i, 1] - Agent.StepRanges[i, 0]) * Rng.NextDouble();
                    ch.Genes[i] *= coeff;
                }

                double[] dq = ch.GetParamAll(Decode);
                Contestant.q = Agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray();
                double dist_new = Contestant.DistanceTo(Goal);
                if (dist_new < dist)
                {
                    Array.Copy(Contestant.q, Agent.q, ParamNum);
                    Min = dist = dist_new;
                    coeff = dist / init_dist;
                }

                if (dist < 0.0002)
                {
                    Converged = true;
                    break;
                }
            }
            sw.Stop();
            Console.WriteLine("IKP Time: {0}; Real time: {1}", Time, sw.ElapsedTicks / 10);

            bool[] Collisions = DetectCollisions(Agent);

            Time = 0;
            return new Tuple<bool, double, double[], bool[]>(Converged, Min, Agent.q.Zip(q_init, (t, s) => { return t - s; }).ToArray(), Collisions);
        }

        public static void FitnessFunction()
        {
            Manipulator Contestant = new Manipulator(Agent);
            for (int i = 0; i < Chs.Length; i++)
            {
                //extracting parameters' values from chromosome
                double[] dq = Chs[i].GetParamAll(Decode);
                Contestant.q = Agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray();

                //applying fitness functions to the given chromosome
                Fit[i] = Contestant.DistanceTo(Goal);
            }
        }
    }

    class PathPlanner : Algorithm
    {
        public static Point Goal;
        public static double Precision;
        public static int ParamNum, GenSize, Count, MaxTime, Time = 0, TotalRealTime = 0;
        public static double Pm;
        public static Chromosome<Point>[] Chs;
        public static int[] Fit;
        public static double[][][] Configs;
        public static Func<Point, Point> Decode;

        public IKP Solver;

        public PathPlanner(double precision, int paramNum, int genSize, double p_m, int maxTime, IKP solver)
        {
            Precision = precision;
            ParamNum = (int)(precision * paramNum);
            GenSize = genSize;
            Pm = p_m;
            MaxTime = maxTime;

            Solver = solver;
            
            Decode = t => t;
        }

        public Tuple<bool, double, Point[], double[][]> Execute(Point goal)
        {
            Goal = goal;

            Segment seg = new Segment(Agent.GripperPos, Goal);
            var Init = new Chromosome<Point>(seg.Discretize(ParamNum - 1));

            Chs = new Chromosome<Point>[GenSize];
            Fit = new int[GenSize];
            Configs = new double[GenSize][][];
            for (int i = 0; i < GenSize; i++)
            {
                Configs[i] = new double[ParamNum][];
            }

            for (int i = 0; i < GenSize; i++)
            {
                Chs[i] = new Chromosome<Point>(ParamNum);
                Chs[i].Genes[0] = Init.Genes[0];
                for (int j = 1; j < ParamNum; j++)
                {
                    Vector vec = new Vector(Init.Genes[j - 1], Init.Genes[j]);
                    vec.Angle += -Math.PI / 2 + Rng.NextDouble() * Math.PI;
                    Chs[i].Genes[j] = Chs[i].Genes[j - 1] + vec;
                }
                //Chs[i] = new Chromosome<Point>(Array.ConvertAll(Init.Genes, t => new Point(t.x + -5 + 10 * Rng.NextDouble(), t.y + -5 + 10 * Rng.NextDouble())));
            }

            Chromosome<Point> dominant = null;
            double[][] Config = null;
            double Max = double.PositiveInfinity;
            bool Converged = false;
            Stopwatch sw = new Stopwatch();
            sw.Start();
            while (Time++ < MaxTime)
            {
                //fitting
                FitnessFunction(Solver);

                //qualification
                Max = Fit.Max();
                if (Max >= Precision * ParamNum)
                {
                    int max_index = Array.IndexOf(Fit, (int)Max);
                    dominant = Chs[max_index];
                    Config = Configs[max_index];
                    Converged = true;
                    break;
                }

                //selection
                RouletteWheelSelection(Chs, Array.ConvertAll(Fit, t => (double)t));

                //mutation
                Mutation(Chs, Pm, t => new Point(t.x + -0.01 + 0.02 * Rng.NextDouble(), t.y + -0.01 + 0.02 * Rng.NextDouble()));

                //crossover
                Crossover(Chs);
            }
            sw.Stop();
            TotalRealTime += (int)sw.ElapsedTicks / 10;
            Console.WriteLine("PP Time: {0}; Real time: {1}", Time, sw.ElapsedTicks / 10);

            //if dominant chromosome wasn't found, consider last result the best result
            if (dominant == null)
            {
                int max_index = Array.IndexOf(Fit, (int)Max);
                dominant = Chs[max_index];
                Config = Configs[max_index];
            }

            //decoding chromosome
            Point[] Points = dominant.GetParamAll(Decode);

            Time = 0;
            return new Tuple<bool, double, Point[], double[][]>(Converged, Max, Points, Config);
        }

        public static void FitnessFunction(IKP solver)
        {
            for (int i = 0; i < Chs.Length; i++)
            {
                Fit[i] = 0;
                for (int j = 0; j < Chs[i].ParamNum; j++)
                {
                    var res = solver.Execute(Chs[i].Genes[j]);
                    if ((res.Item1 || res.Item2 <= 2 * IKP.Precision) && !res.Item4.Contains(true))
                    {
                        Fit[i]++;
                        Configs[i][j] = res.Item3;
                    }
                    else
                        break;
                }
            }
        }

        public static void RouletteWheelSelection(Chromosome<Point>[] Chs, double[] Fit)
        {
            Chromosome<Point>[] selection = new Chromosome<Point>[Chs.Length];
            double total = 0;
            for (int i = 0; i < Chs.Length; i++)
            {
                total += Fit[i];
            }

            double[] sectors = new double[Chs.Length];
            for (int i = 0; i < Chs.Length; i++)
                sectors[i] = Fit[i] / total * 100;

            //randomly selecting crossing chromosomes according to their weights
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

        public static void Crossover(Chromosome<Point>[] Chs)
        {
            Chromosome<Point>[] crossed = new Chromosome<Point>[Chs.Length];

            //forming pairs
            int[] pairs = new int[Chs.Length];
            for (int i = 0; i < Chs.Length; i++)
                pairs[i] = i;

            //shuffling array of pairs to make them random
            int n = pairs.Length;
            while (n > 1)
            {
                int k = Rng.Next(n--);
                int t = pairs[n];
                pairs[n] = pairs[k];
                pairs[k] = t;
            }

            //crossing
            for (int i = 0; i < pairs.Length / 2; i++)
            {
                int point = Rng.Next(0, Chs[0].ParamNum - 1);
                Point[] Ch1 = new Point[Chs[0].ParamNum], Ch2 = new Point[Chs[0].ParamNum];
                for (int j = 0; j < Chs[0].ParamNum; j++)
                {
                    if (j <= point)
                    {
                        Ch1[j] = Chs[pairs[2 * i]].Genes[j];
                        Ch2[j] = Chs[pairs[2 * i + 1]].Genes[j];
                    }
                    else
                    {
                        Vector vec1 = new Vector(Chs[pairs[2 * i]].Genes[j - 1], Chs[pairs[2 * i]].Genes[j]),
                            vec2 = new Vector(Chs[pairs[2 * i + 1]].Genes[j - 1], Chs[pairs[2 * i + 1]].Genes[j]);
                        //Ch1[j] = Chs[pairs[2 * i + 1]].Genes[j];
                        //Ch2[j] = Chs[pairs[2 * i]].Genes[j];
                        Ch1[j] = Ch1[j - 1] + vec2;
                        Ch2[j] = Ch2[j - 1] + vec1;
                    }
                }

                crossed[2 * i] = new Chromosome<Point>(Ch1);
                crossed[2 * i + 1] = new Chromosome<Point>(Ch2);
            }

            crossed.CopyTo(Chs, 0);
        }
    }
}