using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Assimp;
using Logic.InverseKinematics;
using MoreLinq;
using OpenTK;
using Physics;

using Vector3 = System.Numerics.Vector3;
using Vector2 = OpenTK.Vector2;
using System.Drawing;
using System.Threading.Tasks;

namespace Logic.PathPlanning
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;
    using Chromosome = ValueTuple<BezierCurve, Path, float>;

    public enum OptimizationMode
    {
        Maximum,
        Minimum
    }

    [Flags]
    public enum OptimizationCriterion
    {
        GoalConvergence = 0,
        CollisionFree = 1,
        PathLength = 2,
        PathSmoothness = 4
    }

    public enum SelectionMode
    {
        RouletteWheel,
        NormalDistribution
    }

    public enum CrossoverMode
    {
        CrissCross,
        WeightedMean
    }

    internal class ChromosomeD  // TODO: the weight (fit) of the chromosome should be stored inside it
    {
        public int PointsNum;
        public (Vector3, VectorFloat)[] Genes;

        public ChromosomeD(int pointsNum)
        {
            PointsNum = pointsNum;
            Genes = new (Vector3, VectorFloat)[pointsNum];
        }

        /*public Chromosome(T[] genes)
        {
            ParamNum = genes.Length;
            Genes = genes;
        }

        public T GetParam(int Vector3, int param)
        {
            return Genes[Vector3][param];
        }*/

        public (Vector3, VectorFloat) GetGene(int point)
        {
            return Genes[point];
        }

        /*public S[] GetGene<S>(int Vector3, Func<T, S> func)
        {
            S[] _params = new S[ParamNum];
            for (int i = 0; i < ParamNum; i++)
                _params[i] = func(GetParam(Vector3, i));

            return _params;
        }*/
    }

    public partial class GeneticAlgorithm : PathPlanner
    {
        private int _generationSize;
        public ref int GenerationSize => ref _generationSize;

        private float _crossoverProbability;
        public ref float CrossoverProbability => ref _crossoverProbability;

        private float _mutationProbability;
        public ref float MutationProbability => ref _mutationProbability;

        private OptimizationMode _optimizationMode;
        public ref OptimizationMode OptimizationMode => ref _optimizationMode;

        private OptimizationCriterion _optimizationCriteria = OptimizationCriterion.CollisionFree;
        public ref OptimizationCriterion OptimizationCriteria => ref _optimizationCriteria;

        private SelectionMode _selectionMode = SelectionMode.NormalDistribution;
        public ref SelectionMode SelectionMode => ref _selectionMode;

        private CrossoverMode _crossoverMode = CrossoverMode.WeightedMean;
        public ref CrossoverMode CrossoverMode => ref _crossoverMode;

        private Manipulator _agent, _agentCopy;
        private InverseKinematicsSolver _solver;
        private Vector3 _goal;

        public static Chromosome Dominant;
        public static volatile bool Locked;
        public static volatile bool Changed;

        private int offspringSize = 4;
        private int survivalSize = 10;
        private int bezierCount = 2;
        private float bezierStep = 0.015f;

        private float sqrt2pi = 1 / (float)Math.Sqrt(2 * Math.PI);

        public GeneticAlgorithm(int maxTime, bool collisionCheck, 
            int generationSize, float crossoverProbability, float mutationProbability) : base(maxTime, collisionCheck)
        {
            _generationSize = generationSize;
            _crossoverProbability = crossoverProbability;
            _mutationProbability = mutationProbability;
        }

        protected override (int, Path) RunAbstract(Manipulator agent, Vector3 goal, InverseKinematicsSolver solver)
        {
            _agent = agent;
            _goal = goal;
            _solver = solver;

            using (var agentCopy = agent.DeepCopy())
            {
                _agentCopy = agentCopy;

                int generationsCount = 0;
                var generation = new List<Chromosome>();

                // get initial solution
                var bezierInitial = ConstructBezier(new Vector2(_agentCopy.GripperPos.Z, _agentCopy.GripperPos.Y), new Vector2(goal.Z, goal.Y), bezierCount);
                var pathInitial = ConstructPath(bezierInitial, bezierStep);
                Chromosome chromosomeInitial = (bezierInitial, pathInitial, Fit(pathInitial));

                generation.Add(chromosomeInitial);

                Dominant = chromosomeInitial;

                // evolve generations
                Console.SetCursorPosition(0, 10);
                Console.WriteLine("Starting genetic algorithm...");
                while (generationsCount++ < MaxTime)
                {
                    // get new generation
                    generation = Evolve(generation, offspringSize, survivalSize);

                    Console.SetCursorPosition(0, 11);
                    Console.WriteLine($"Generations passed: {generationsCount}");
                    Console.WriteLine($"Best fit: {generation[0].Item3}");

                    if (!Locked)
                    {
                        //if (generation[0].Item3 < Dominant.Item3)
                        {
                            Dominant = generation[0];
                            Changed = true;
                        }
                    }

                    // break if max fit reached
                    if (generation[0].Item3 == 0)
                        break;
                }

                Console.SetCursorPosition(0, 15);
                Console.WriteLine("Found path's Bezier curve:");
                foreach (var point in generation[0].Item1.Points)
                {
                    Console.WriteLine(point);
                }

                return (generationsCount - 1, generation[0].Item2);
            }
        }

        private BezierCurve ConstructBezier(Vector3 start, Vector3 end, int intermediaryPoints)
        {
            Vector3[] points = new Vector3[2 + intermediaryPoints];

            Vector3 direction = end - start;
            Vector3 directionNorm = direction.Normalized();
            float length = direction.Length;
            float segmentCountInv = 1.0f / (points.Length - 1);

            points[0] = start;
            for (int i = 1; i < points.Length - 1; i++)
            {
                points[i] = start + directionNorm * i * length * segmentCountInv;
            }
            points[points.Length - 1] = end;

            return new BezierCurve(points);
        }

        private Path ConstructPath(BezierCurve bezierCurve, float step)
        {
            // reset agent
            _agentCopy.q = _agent.q;

            float counter = 0;

            var points = new List<Vector3[]> { _agentCopy.DKP };
            var configs = new List<VectorFloat> { _agentCopy.q };
            while (counter <= 1)
            {
                var bezierPoint = bezierCurve.CalculatePoint(counter);
                _solver.Execute(_agentCopy, new Vector3(0, bezierPoint.Y, bezierPoint.X), _agentCopy.Joints.Length - 1);
                points.Add(_agentCopy.DKP);
                configs.Add(_agentCopy.q);

                counter += step;
            }

            return new Path(points, configs);
        }

        private List<Vector3> BezierPoints(BezierCurve bezierCurve, float step)
        {
            var points = new List<Vector3>();

            float counter = 0;
            while (counter <= 1)
            {
                var bezierPoint = bezierCurve.CalculatePoint(counter);
                points.Add(new Vector3(0, bezierPoint.Y, bezierPoint.X));
                counter += step;
            }

            return points;
        }

        private List<Chromosome> Evolve(List<Chromosome> generation, int offspringSize, int survivalSize)
        {
            var offsprings = new List<Chromosome>();
            foreach (var member in generation)
            {
                offsprings.AddRange(Reproduce(member, offspringSize));
            }

            //Rate(offsprings);

            return Select(offsprings, survivalSize);
        }

        //private void Rate(List<Chromosome> generation)
        //{
        //    for (int i = 0; i < generation.Count; i++)
        //    {
        //        generation[i] = (generation[i].Item1, generation[i].Item2, Fit(generation[i].Item2));
        //    }
        //}

        private BezierCurve Repro(BezierCurve bezierCurve)
        {
            int pointIndex = RandomThreadStatic.Next(1, 3);
            var z = RandomThreadStatic.NextDouble(1);
            var y = RandomThreadStatic.NextDouble(1);
            Vector2 direction = new Vector2((float)z, (float)y);

            var bezierCurveRepro = new BezierCurve(bezierCurve.Points);
            var bezierDirectionRepro = direction.Normalized() * 1f * (float)RandomThreadStatic.NextDouble();

            bezierCurveRepro.Points[pointIndex] += bezierDirectionRepro;

            if ((bezierCurveRepro.Points[pointIndex] + bezierDirectionRepro).Length < _agentCopy.WorkspaceRadius)
                bezierCurveRepro.Points[pointIndex] += bezierDirectionRepro;
            else
                bezierCurveRepro.Points[pointIndex] -= bezierDirectionRepro;

            return bezierCurveRepro;
        }

        private List<Chromosome> Reproduce(Chromosome member, int offspringSize)
        {
            var repro = new List<Chromosome>();
            for (int i = 0; i < offspringSize; i++)
            {
                BezierCurve bezierCurveRepro = Repro(member.Item1);
                Path pathRepro = ConstructPath(bezierCurveRepro, bezierStep);
                repro.Add((bezierCurveRepro, pathRepro, Fit(pathRepro)));
            }

            return repro;
        }

        private List<Chromosome> Select(List<Chromosome> offsprings, int survivalSize)
        {
            return offsprings.OrderBy(x => x.Item3).Take(survivalSize).ToList();
        }

        private float Fit(Path sample)
        {
            float score = 0;

            // extract parameters' values from chromosome
            foreach (var node in sample.Nodes)
            {
                _agentCopy.q = node.q;

                Vector3 currPos = node.Points[node.Points.Length - 1];

                // apply fitness functions to the given chromosome's point
                int criteriaCount = 0;
                float pointWeight = 0;

                if (_optimizationCriteria.HasFlag(OptimizationCriterion.CollisionFree))
                {
                    // TODO: instead of weighting, add "pulling-out", i.e. translating point along the common vector with the obst center; much better
                    foreach (var obst in ObstacleHandler.Obstacles)
                    {
                        if (obst.Contains(currPos))
                        {
                            //var collider = obst.Collider as SphereCollider;
                            //var centerOfMass = collider.Body.CenterOfMassPosition;
                            //var center = new Vector3(centerOfMass.X, centerOfMass.Y, centerOfMass.Z);
                            //pointWeight += 100 * (float)Math.Pow(currPos.DistanceTo(center) / collider.Radius, 4);
                            pointWeight += 1;
                        }
                        else
                        {

                        }
                    }

                    if (_agentCopy.CollisionTest().Contains(true))
                    {
                        pointWeight += 1;
                    }

                    criteriaCount++;
                }
                if (_optimizationCriteria.HasFlag(OptimizationCriterion.PathLength))
                {

                }
                if (_optimizationCriteria.HasFlag(OptimizationCriterion.PathSmoothness))
                {

                }

                // take median of all criteria weights
                score += pointWeight / criteriaCount;
            }

            return score;
        }

        #region USABLE
        //protected override (int, Path) RunAbstract(Manipulator agent, Vector3 goal, InverseKinematicsSolver solver)
        //{
        //    _agent = agent;
        //    _goal = goal;
        //    _solver = solver;

        //    using (var agentCopy = agent.DeepCopy())
        //    {
        //        _agentCopy = agentCopy;

        //        int generationsCount = 0;
        //        var generation = new List<Chromosome>();

        //        // get initial solution
        //        var bezierInitial = ConstructBezier(new Vector2(_agentCopy.GripperPos.Z, _agentCopy.GripperPos.Y), new Vector2(goal.Z, goal.Y), bezierCount);
        //        var pathInitial = ConstructPath(bezierInitial, bezierStep);
        //        Chromosome chromosomeInitial = (bezierInitial, pathInitial, Fit(pathInitial));

        //        generation.Add(chromosomeInitial);

        //        Dominant = chromosomeInitial;

        //        // evolve generations
        //        Console.SetCursorPosition(0, 10);
        //        Console.WriteLine("Starting genetic algorithm...");
        //        while (generationsCount++ < MaxTime)
        //        {
        //            // get new generation
        //            generation = Evolve(generation, offspringSize, survivalSize);

        //            Console.SetCursorPosition(0, 11);
        //            Console.WriteLine($"Generations passed: {generationsCount}");
        //            Console.WriteLine($"Best fit: {generation[0].Item3}");

        //            if (!Locked)
        //            {
        //                //if (generation[0].Item3 < Dominant.Item3)
        //                {
        //                    Dominant = generation[0];
        //                    Changed = true;
        //                }
        //            }

        //            // break if max fit reached
        //            if (generation[0].Item3 == 0)
        //                break;
        //        }

        //        Console.SetCursorPosition(0, 15);
        //        Console.WriteLine("Found path's Bezier curve:");
        //        foreach (var point in generation[0].Item1.Points)
        //        {
        //            Console.WriteLine(point);
        //        }

        //        return (generationsCount - 1, generation[0].Item2);
        //    }
        //}

        //private BezierCurve ConstructBezier(Vector2 start, Vector2 end, int intermediaryPoints)
        //{
        //    Vector2[] points = new Vector2[2 + intermediaryPoints];

        //    Vector2 direction = end - start;
        //    Vector2 directionNorm = direction.Normalized();
        //    float length = direction.Length;
        //    float segmentCountInv = 1.0f / (points.Length - 1);

        //    points[0] = start;
        //    for (int i = 1; i < points.Length - 1; i++)
        //    {
        //        points[i] = start + directionNorm * i * length * segmentCountInv;
        //    }
        //    points[points.Length - 1] = end;

        //    return new BezierCurve(points);
        //}

        //private Path ConstructPath(BezierCurve bezierCurve, float step)
        //{
        //    // reset agent
        //    _agentCopy.q = _agent.q;

        //    float counter = 0;

        //    var points = new List<Vector3[]> { _agentCopy.DKP };
        //    var configs = new List<VectorFloat> { _agentCopy.q };
        //    while (counter <= 1)
        //    {
        //        var bezierPoint = bezierCurve.CalculatePoint(counter);
        //        _solver.Execute(_agentCopy, new Vector3(0, bezierPoint.Y, bezierPoint.X), _agentCopy.Joints.Length - 1);
        //        points.Add(_agentCopy.DKP);
        //        configs.Add(_agentCopy.q);

        //        counter += step;
        //    }

        //    return new Path(points, configs);
        //}

        //private List<Vector3> BezierPoints(BezierCurve bezierCurve, float step)
        //{
        //    var points = new List<Vector3>();

        //    float counter = 0;
        //    while (counter <= 1)
        //    {
        //        var bezierPoint = bezierCurve.CalculatePoint(counter);
        //        points.Add(new Vector3(0, bezierPoint.Y, bezierPoint.X));
        //        counter += step;
        //    }

        //    return points;
        //}

        //private List<Chromosome> Evolve(List<Chromosome> generation, int offspringSize, int survivalSize)
        //{
        //    var offsprings = new List<Chromosome>();
        //    foreach (var member in generation)
        //    {
        //        offsprings.AddRange(Reproduce(member, offspringSize));
        //    }

        //    //Rate(offsprings);

        //    return Select(offsprings, survivalSize);
        //}

        ////private void Rate(List<Chromosome> generation)
        ////{
        ////    for (int i = 0; i < generation.Count; i++)
        ////    {
        ////        generation[i] = (generation[i].Item1, generation[i].Item2, Fit(generation[i].Item2));
        ////    }
        ////}

        //private BezierCurve Repro(BezierCurve bezierCurve)
        //{
        //    int pointIndex = RandomThreadStatic.Next(1, 3);
        //    var z = RandomThreadStatic.NextDouble(1);
        //    var y = RandomThreadStatic.NextDouble(1);
        //    Vector2 direction = new Vector2((float)z, (float)y);

        //    var bezierCurveRepro = new BezierCurve(bezierCurve.Points);
        //    var bezierDirectionRepro = direction.Normalized() * 1f * (float)RandomThreadStatic.NextDouble();

        //    bezierCurveRepro.Points[pointIndex] += bezierDirectionRepro;

        //    if ((bezierCurveRepro.Points[pointIndex] + bezierDirectionRepro).Length < _agentCopy.WorkspaceRadius)
        //        bezierCurveRepro.Points[pointIndex] += bezierDirectionRepro;
        //    else
        //        bezierCurveRepro.Points[pointIndex] -= bezierDirectionRepro;

        //    return bezierCurveRepro;
        //}

        //private List<Chromosome> Reproduce(Chromosome member, int offspringSize)
        //{
        //    var repro = new List<Chromosome>();
        //    for (int i = 0; i < offspringSize; i++)
        //    {
        //        BezierCurve bezierCurveRepro = Repro(member.Item1);
        //        Path pathRepro = ConstructPath(bezierCurveRepro, bezierStep);
        //        repro.Add((bezierCurveRepro, pathRepro, Fit(pathRepro)));
        //    }

        //    return repro;
        //}

        //private List<Chromosome> Select(List<Chromosome> offsprings, int survivalSize)
        //{
        //    return offsprings.OrderBy(x => x.Item3).Take(survivalSize).ToList();
        //}

        //private float Fit(Path sample)
        //{
        //    float score = 0;

        //    // extract parameters' values from chromosome
        //    foreach (var node in sample.Nodes)
        //    {
        //        _agentCopy.q = node.q;

        //        Vector3 currPos = node.Points[node.Points.Length - 1];

        //        // apply fitness functions to the given chromosome's point
        //        int criteriaCount = 0;
        //        float pointWeight = 0;

        //        if (_optimizationCriteria.HasFlag(OptimizationCriterion.CollisionFree))
        //        {
        //            // TODO: instead of weighting, add "pulling-out", i.e. translating point along the common vector with the obst center; much better
        //            foreach (var obst in ObstacleHandler.Obstacles)
        //            {
        //                if (obst.Contains(currPos))
        //                {
        //                    //var collider = obst.Collider as SphereCollider;
        //                    //var centerOfMass = collider.Body.CenterOfMassPosition;
        //                    //var center = new Vector3(centerOfMass.X, centerOfMass.Y, centerOfMass.Z);
        //                    //pointWeight += 100 * (float)Math.Pow(currPos.DistanceTo(center) / collider.Radius, 4);
        //                    pointWeight += 1;
        //                }
        //                else
        //                {

        //                }
        //            }

        //            if (_agentCopy.CollisionTest().Contains(true))
        //            {
        //                pointWeight += 1;
        //            }

        //            criteriaCount++;
        //        }
        //        if (_optimizationCriteria.HasFlag(OptimizationCriterion.PathLength))
        //        {

        //        }
        //        if (_optimizationCriteria.HasFlag(OptimizationCriterion.PathSmoothness))
        //        {

        //        }

        //        // take median of all criteria weights
        //        score += pointWeight / criteriaCount;
        //    }

        //    return score;
        //}
        #endregion

        #region new
        //public override Path Execute(Obstacle[] obstacles, Manipulator agent, Vector3 goal, InverseKinematicsSolver solver/*, Func<float, float> decode)
        //{
        //    #region old
        //    //var chs = new ChromosomeD[_generationSize];
        //    //var fit = new float[_generationSize];

        //    //int pointsNum = _initialSolution.Length;
        //    //for (int i = 0; i < _generationSize; i++)
        //    //{
        //    //    chs[i] = new ChromosomeD(pointsNum);

        //    //    // define distortion amplitude as a normal distribution
        //    //    int mu = RandomThreadStatic.Next(1, _initialSolution.Length - 1);
        //    //    float sigma = 10;
        //    //    float amp = -0.5f + 1 * (float)RandomThreadStatic.NextDouble();
        //    //    float nd(int t) => amp * (float)Math.Exp(-0.5 * Math.Pow((t - mu) / sigma, 2));

        //    //    // segment between prev and next points
        //    //    Vector3 vec = _initialSolution.[mu + 1].Item1 - _initialSolution[mu - 1].Item1;

        //    //    // get distortion direction
        //    //    var x = RandomThreadStatic.NextDouble();
        //    //    var y = RandomThreadStatic.NextDouble();
        //    //    var z = (vec.X * x + vec.Y * y) / vec.Z;
        //    //    var dir = Vector3.Normalize(new Vector3((float)x, (float)y, (float)z));

        //    //    // apply distortion to all points except start and end
        //    //    chs[i].Genes[0].Item1 = _initialSolution[0].Item1;
        //    //    for (int j = 1; j < pointsNum - 1; j++)
        //    //    {
        //    //        chs[i].Genes[j].Item1 = _initialSolution[j].Item1 + dir * nd(j);
        //    //    }
        //    //    chs[i].Genes[_initialSolution.Length - 1].Item1 = initSolution[_initialSolution.Length - 1].Item1;

        //    //    /*if (i == 0)
        //    //    {
        //    //        agent.Vector3s = chs[i].Genes.Select(t => t.Item1).ToArray();
        //    //    }*/
        //    //}

        //    //ChromosomeD dominant = null;
        //    //float max = 0;
        //    //bool converged = false;
        //    //int time = 0;
        //    //Dispatcher.Timer.Start();
        //    //while (time++ < MaxTime)
        //    //{
        //    //    if (time % 100 == 0)
        //    //    {

        //    //    }

        //    //    // fitting
        //    //    FitnessFunctionD(agent, obstacles, goal, solver, chs, fit, _optimizationCriteria/*, decode*/);

        //    //    // qualification
        //    //    max = fit.Max();

        //    //    dominant = chs[Array.IndexOf(fit, max)];
        //    //    //agent.Vector3s = dominant.Genes.Select(t => t.Item1).ToArray();

        //    //    Console.SetCursorPosition(0, 10);
        //    //    Console.WriteLine($"Goal fit: {pointsNum * 100}");
        //    //    Console.WriteLine($"Max  fit: {max}");
        //    //    Console.WriteLine("GA Time: {0}; Real time: {1}", time, Dispatcher.Timer.ElapsedTicks / 10);
        //    //    //if (max > precision * Vector3sNum * 100)
        //    //    if (max >= pointsNum * 100)
        //    //    {
        //    //        dominant = chs[Array.IndexOf(fit, max)];
        //    //        converged = true;
        //    //        break;
        //    //    }

        //    //    // selection
        //    //    SelectionD(chs, fit, _selectionMode, OptimizationMode.Maximum);

        //    //    // mutation
        //    //    MutationD(chs, _mutationProbability, t => t + -0.1f + 0.2f * (float)RandomThreadStatic.NextDouble());

        //    //    // crossover
        //    //    CrossoverD(chs, fit, _crossoverProbability, _crossoverMode);
        //    //}
        //    //Dispatcher.Timer.Reset();

        //    //// if the dominant chromosome wasn't found, consider last result the best result
        //    //if (dominant == null)  // TODO: at this point fit and chs arrays do not correlate, because chs was changed!
        //    //{
        //    //    dominant = chs[Array.IndexOf(fit, max)];
        //    //}

        //    //Manipulator agentCopy = agent.DeepCopy();
        //    //Dispatcher.Timer.Start();
        //    //for (int i = 0; i < dominant.PointsNum; i++)
        //    //{
        //    //    if (i == 0)
        //    //    {
        //    //        agent.q.CopyTo(dominant.Genes[i].Item2);
        //    //    }
        //    //    else
        //    //    {
        //    //        dominant.Genes[i - 1].Item2.CopyTo(agentCopy.q);
        //    //        var res = solver.Execute(obstacles, agentCopy, dominant.Genes[i].Item1, agent.Joints.Length - 1);

        //    //        // assign config to continue checking other points
        //    //        agentCopy.q.CopyTo(dominant.Genes[i].Item2);
        //    //    }
        //    //}
        //    //Dispatcher.Timer.Stop();
        //    //Console.WriteLine("Fit real time: {0}", Dispatcher.Timer.ElapsedTicks / 10);
        //    //Dispatcher.Timer.Reset();

        //    //// chromosome
        //    //List<Vector3> path = new List<Vector3>();
        //    //List<VectorFloat> configs = new List<VectorFloat>();
        //    //for (int i = 0; i < pointsNum; i++)
        //    //{
        //    //    path.Add(dominant.Genes[i].Item1);
        //    //    configs.Add(dominant.Genes[i].Item2);
        //    //}

        //    //// detect all collisions
        //    ///*Manipulator AgentNext = new Manipulator(agent)
        //    //{
        //    //    q = agent.q.Zip(dq, (t, s) => { return t + s; }).ToArray()
        //    //};
        //    //bool[] Collisions = DetectCollisions(AgentNext);*/

        //    //return (path, configs);
        //    #endregion

        //    // algorithm parameters
        //    int offspringSize = 4, survivalSize = 10;

        //    _agentCopy = agent.DeepCopy();
        //    _goal = goal;
        //    _solver = solver;

        //    int genCount = 0;
        //    var generation = new List<(Path, float)>();

        //    // get initial solution
        //    var points = new List<Vector3[]> { _agentCopy.DKP };
        //    var configs = new List<VectorFloat> { _agentCopy.q };
        //    while (_agentCopy.GripperPos.DistanceTo(goal) > 0.04)
        //    {
        //        _solver.Execute(obstacles, _agentCopy, _agentCopy.GripperPos + Vector3.Normalize(_goal - _agentCopy.GripperPos) * 0.04f, _agentCopy.Joints.Length - 1);
        //        points.Add(_agentCopy.DKP);
        //        configs.Add(_agentCopy.q);
        //    }

        //    Path initialSolution = new Path(points, configs);
        //    generation.Add((initialSolution, Fit(initialSolution)));

        //    Dominant = initialSolution;

        //    // evolve generations
        //    while (genCount++ < MaxTime)
        //    {
        //        // get new generation
        //        generation = Evolve(generation, offspringSize, survivalSize);

        //        Console.SetCursorPosition(0, 10);
        //        Console.WriteLine(generation[0].Item2);

        //        if (!Locked)
        //        {
        //            Dominant.AddBuffer = generation[0].Item1.AddBuffer;
        //            Dominant.DelBuffer = generation[0].Item1.DelBuffer;
        //            Changed = true;
        //        }

        //        // break if max fit reached
        //        if (generation[0].Item1.Count * 100 == generation[0].Item2)
        //            break;
        //    }

        //    return generation[0].Item1;
        //}

        //private List<(Path, float)> Evolve(List<(Path, float)> generation, int offspringSize, int survivalSize)
        //{
        //    var offsprings = new List<(Path, float)>();
        //    foreach (var member in generation)
        //    {
        //        offsprings.AddRange(Reproduce(member, offspringSize));
        //    }

        //    return Select(offsprings, survivalSize);
        //}

        //private List<(Path, float)> Reproduce((Path, float) member, int offspringSize)
        //{
        //    void UpdateNode(Path.Node node, Vector3 goal)
        //    {
        //        _agentCopy.q = node.q;
        //        _solver.Execute(null, _agentCopy, goal, _agentCopy.Joints.Length - 1);
        //        node.q = _agentCopy.q;
        //        node.Points = _agentCopy.DKP;
        //    }

        //    var repro = new List<(Path, float)>();
        //    for (int i = 0; i < offspringSize; i++)
        //    {
        //        // create and add new offspring
        //        var path = member.Item1.DeepCopy();

        //        // get the affected node
        //        int index = RandomThreadStatic.Next(1, path.Count - 1);
        //        Path.Node nodeRandom = path.Nodes.ElementAt(index);
        //        if (nodeRandom.Parent == null)
        //            nodeRandom = nodeRandom.Child;
        //        else if (nodeRandom.Child == null)
        //            nodeRandom = nodeRandom.Parent;

        //        // define distortion amplitude as a normal distribution
        //        float sigma = 5;
        //        float amp = 1/* - (float)RandomThreadStatic.NextDouble()*/;
        //        float nd(int t) => amp * (1 / (sigma * sqrt2pi)) * (float)Math.Exp(-0.5 * Math.Pow(t / sigma, 2));

        //        // segment between previous and next points
        //        var count = nodeRandom.Points.Length;
        //        Vector3 vec = nodeRandom.Points[count - 1] - nodeRandom.Parent.Points[count - 1];

        //        // get distortion direction
        //        var x = RandomThreadStatic.NextDouble();
        //        var y = RandomThreadStatic.NextDouble();
        //        var z = -(vec.X * x + vec.Y * y) / vec.Z;
        //        var dir = Vector3.Normalize(new Vector3((float)x, (float)y, (float)z));

        //        // apply distortion to the current point
        //        Path.Node current = nodeRandom;
        //        UpdateNode(current, current.Points[count - 1] + dir * nd(0));

        //        // apply distortion to previous points
        //        int pointsNum = 20;
        //        current = nodeRandom.Parent;
        //        for (int j = -1; j >= -pointsNum && current.Parent != null; j--)
        //        {
        //            UpdateNode(current, current.Points[count - 1] + dir * nd(j));
        //            current = current.Parent;
        //        }

        //        // apply distortion to next points
        //        current = nodeRandom.Child;
        //        for (int j = 1; j <= pointsNum && current.Child != null; j++)
        //        {
        //            UpdateNode(current, current.Points[count - 1] + dir * nd(j));
        //            current = current.Child;
        //        }

        //        repro.Add((path, Fit(path)));
        //    }

        //    return repro;
        //}

        //private List<(Path, float)> Select(List<(Path, float)> offsprings, int survivalSize)
        //{
        //    return offsprings.OrderBy(x => x.Item2).Take(survivalSize).ToList();
        //}

        //private float Fit(Path sample)
        //{
        //    float score = 0;

        //    // extract parameters' values from chromosome
        //    foreach (var node in sample.Nodes)
        //    {
        //        Vector3 currPos = node.Points[node.Points.Length - 1];

        //        // apply fitness functions to the given chromosome's point
        //        int criteriaCount = 0;
        //        float pointWeight = 0;

        //        if (_optimizationCriteria.HasFlag(OptimizationCriterion.CollisionFree))
        //        {
        //            // TODO: instead of weighting, add "pulling-out", i.e. translating point along the common vector with the obst center; much better
        //            foreach (var obst in ObstacleHandler.Obstacles)
        //            {
        //                if (obst.Contains(currPos))
        //                {
        //                    var collider = obst.Collider as SphereCollider;
        //                    var centerOfMass = collider.Body.CenterOfMassPosition;
        //                    var center = new Vector3(centerOfMass.X, centerOfMass.Y, centerOfMass.Z);
        //                    pointWeight += 100 * (float)Math.Pow(currPos.DistanceTo(center) / collider.Radius, 4);
        //                }
        //                else
        //                {
        //                    pointWeight += 100;
        //                }
        //            }
        //            criteriaCount++;
        //        }
        //        if (_optimizationCriteria.HasFlag(OptimizationCriterion.PathLength))
        //        {

        //        }
        //        if (_optimizationCriteria.HasFlag(OptimizationCriterion.PathSmoothness))
        //        {

        //        }

        //        // take median of all criteria weights
        //        score += pointWeight / criteriaCount;
        //    }

        //    return score;
        //}
        #endregion

        #region old
        private static void FitnessFunctionD/*<T>*/(Manipulator agent, Obstacle[] obstacles, Vector3 goal, InverseKinematicsSolver solver,
            ChromosomeD[] chs, float[] fit, OptimizationCriterion criterion/*, Func<T, float> decode*/)
        {
            Manipulator agentCopy = agent.DeepCopy();
            for (int i = 0; i < chs.Length; i++)
            {
                fit[i] = 0;

                agent.q.CopyTo(agentCopy.q);

                // extract parameters' values from chromosome
                for (int j = 0; j < chs[i].PointsNum; j++)
                {
                    Vector3 currPos = chs[i].Genes[j].Item1;

                    // apply fitness functions to the given chromosome's point
                    int critCount = 0;
                    float pointWeight = 0;

                    /*if (j == 0)
                    {
                        chs[i].Genes[j].Item2 = Misc.CopyArray(agent.q);
                        Vector3Weight += 100;
                    }
                    else
                    {
                        for (int k = 0; k < 1; k++)
                        {
                            contestant.q = Misc.CopyArray(chs[i].Genes[j - 1].Item2);
                            var res = solver.Execute(contestant, chs[i].Genes[j].Item1);

                            // assign config to continue checking other Vector3s
                            chs[i].Genes[j].Item2 = Misc.CopyArray(contestant.q);

                            if (res.Item1 && !res.Item4.Contains(true))
                            {
                                Vector3Weight += 100;
                                break;
                            }
                        }
                    }
                    critCount++;*/

                    if ((criterion & OptimizationCriterion.CollisionFree) == OptimizationCriterion.CollisionFree)
                    {
                        // TODO: instead of weighting, add "pulling-out", i.e. translating point along the common vector with the obst center; much better
                        foreach (var obst in obstacles)
                        {
                            if (obst.Contains(currPos))
                            {
                                var collider = obst.Collider as SphereCollider;
                                var centerOfMass = collider.Body.CenterOfMassPosition;
                                var center = new Vector3(centerOfMass.X, centerOfMass.Y, centerOfMass.Z);
                                pointWeight += 100 * (float)Math.Pow(currPos.DistanceTo(center) / collider.Radius, 4);
                            }
                            else
                            {
                                pointWeight += 100;
                            }
                        }
                        critCount++;
                    }
                    if ((criterion & OptimizationCriterion.PathLength) == OptimizationCriterion.PathLength)
                    {

                    }
                    if ((criterion & OptimizationCriterion.PathSmoothness) == OptimizationCriterion.PathSmoothness)
                    {

                    }

                    // take median of all criteria weights
                    fit[i] += pointWeight / critCount;
                }
            }
        }

        private static void SelectionD(ChromosomeD[] chs, float[] fit, SelectionMode selectMode, OptimizationMode optimizeMode)
        {
            ChromosomeD[] selection = new ChromosomeD[chs.Length];

            // select chromosomes with the specified mode
            switch (selectMode)
            {
                case SelectionMode.RouletteWheel:
                    float[] sectors = new float[chs.Length];
                    float total;
                    switch (optimizeMode)
                    {
                        case OptimizationMode.Maximum:
                            total = fit.Sum();
                            for (int i = 0; i < chs.Length; i++)
                                sectors[i] = fit[i] / total * 100;
                            break;
                        case OptimizationMode.Minimum:
                            total = fit.Sum((x) => 1 / x);
                            for (int i = 0; i < chs.Length; i++)
                                sectors[i] = (1 / fit[i]) / total * 100;
                            break;
                    }

                    // randomly select crossing chromosomes according to their weights
                    for (int i = 0; i < chs.Length; i++)
                    {
                        float point = (float)RandomThreadStatic.NextDouble() * 100, seek = 0;

                        for (int j = 0; j < chs.Length; j++)
                        {
                            seek += sectors[j];
                            if (point < seek)
                            {
                                selection[i] = chs[j];
                                break;
                            }
                        }
                    }
                    break;
                case SelectionMode.NormalDistribution:
                    var fitList = fit
                        .Select((x, i) => new KeyValuePair<int, float>(i, x))
                        .OrderBy(x => x.Value)
                        .ToList();
                    var fitValues = fitList.Select(x => x.Value).ToList();
                    var fitKeys = fitList.Select(x => x.Key).ToList();
                    var min = fitValues[0];
                    var max = fitValues[fitValues.Count - 1];
                    float num;
                    int index = 0;

                    switch (optimizeMode)
                    {
                        case OptimizationMode.Maximum:
                            for (int i = 0; i < chs.Length; i++)
                            {
                                num = RandomThreadStatic.NextGaussian(max, (max - min) / 3);

                                if (num <= min)
                                    index = fitKeys[0];
                                else if (num >= max)
                                {
                                    var diff = num - max;
                                    num = max - diff;
                                    index = fitKeys[fitValues.FindIndex(x => x >= num)];
                                }
                                else
                                    index = fitKeys[fitValues.FindIndex(x => x > num)];

                                selection[i] = chs[index];
                            }
                            break;
                        case OptimizationMode.Minimum:
                            for (int i = 0; i < chs.Length; i++)
                            {
                                num = RandomThreadStatic.NextGaussian(max, (max - min) / 3);
                                if (num <= min)
                                {
                                    var diff = num - min;
                                    num = min - diff;
                                    index = fitKeys[fitValues.FindIndex(x => x <= num)];
                                }
                                else if (num >= max)
                                    index = fitKeys[fitValues.Count - 1];
                                else
                                    index = fitKeys[fitValues.FindIndex(x => x < num)];

                                selection[i] = chs[index];
                            }
                            break;
                    }
                    break;
            }

            selection.CopyTo(chs, 0);
        }

        private static void MutationD(ChromosomeD[] chs, float mutationProb, Func<float, float> mutate)
        {
            // randomly choose among chromosomes the mutating ones
            foreach (var chr in chs)
            {
                if (RandomThreadStatic.NextDouble() < mutationProb)
                {
                    // define distortion amplitude as a normal distribution
                    int mu = RandomThreadStatic.Next(1, chr.PointsNum - 1);
                    float sigma = 10;
                    float amp = mutate(0);
                    float nd(int t) => amp * (float)Math.Exp(-0.5 * Math.Pow((t - mu) / sigma, 2));

                    // segment between prev and next Vector3s
                    var vec = chr.Genes[mu + 1].Item1 - chr.Genes[mu - 1].Item1;

                    // get distortion direction
                    var x = RandomThreadStatic.NextDouble();
                    var y = RandomThreadStatic.NextDouble();
                    var z = (vec.X * x + vec.Y * y) / vec.Z;
                    var dir = Vector3.Normalize(new Vector3((float)x, (float)y, (float)z));

                    // slightly change all the points of the chromosome except start and end
                    for (int i = 1; i < chr.PointsNum - 1; i++)
                    {
                        chr.Genes[i].Item1 += dir * nd(i);

                        // pull vertex to its neighbour to retain desired step distance
                        vec = chr.Genes[i].Item1 - chr.Genes[i - 1].Item1;
                        chr.Genes[i].Item1 += (vec.Length() - 0.04f) * Vector3.Normalize(vec);
                    }
                }
            }
        }

        private static void CrossoverD(ChromosomeD[] chs, float[] fit, float crossoverProb, CrossoverMode crossMode)  // TODO: add generic types; add crossover probability Pc
        {
            var crossed = new ChromosomeD[chs.Length];

            // form pairs
            int[] pairs = new int[chs.Length];
            for (int i = 0; i < chs.Length; i++)
                pairs[i] = i;

            // shuffle array of pairs to make them random
            int n = pairs.Length;
            while (n > 1)
            {
                int k = RandomThreadStatic.Next(n--);
                int t = pairs[n];
                pairs[n] = pairs[k];
                pairs[k] = t;
            }

            // crossover with the specified mode according to probability
            switch (crossMode)
            {
                case CrossoverMode.CrissCross:
                    for (int i = 0; i < pairs.Length / 2; i++)
                    {
                        if (RandomThreadStatic.NextDouble() < crossoverProb)
                        {
                            int Vector3 = RandomThreadStatic.Next(0, chs[0].PointsNum - 1);
                            ChromosomeD ch1 = new ChromosomeD(chs[0].PointsNum),
                                        ch2 = new ChromosomeD(chs[0].PointsNum);
                            for (int j = 0; j < chs[0].PointsNum; j++)
                            {
                                if (j <= Vector3)
                                {
                                    ch1.Genes[j] = chs[pairs[2 * i]].Genes[j];
                                    ch2.Genes[j] = chs[pairs[2 * i + 1]].Genes[j];
                                }
                                else
                                {
                                    ch1.Genes[j] = chs[pairs[2 * i + 1]].Genes[j];
                                    ch2.Genes[j] = chs[pairs[2 * i]].Genes[j];
                                }
                            }

                            crossed[2 * i] = ch1;
                            crossed[2 * i + 1] = ch2;
                        }
                        else
                        {
                            crossed[2 * i] = chs[pairs[2 * i]];
                            crossed[2 * i + 1] = chs[pairs[2 * i + 1]];
                        }
                    }
                    break;
                case CrossoverMode.WeightedMean:
                    (Vector3, VectorFloat) G1, G2;
                    float W1, W2;
                    for (int i = 0; i < pairs.Length / 2; i++)
                    {
                        if (RandomThreadStatic.NextDouble() < crossoverProb)
                        {
                            int Vector3 = RandomThreadStatic.Next(0, chs[0].PointsNum - 1);
                            ChromosomeD ch1 = new ChromosomeD(chs[0].PointsNum),
                                        ch2 = new ChromosomeD(chs[0].PointsNum);

                            W1 = fit[pairs[2 * i]];
                            W2 = fit[pairs[2 * i + 1]];
                            for (int j = 0; j < chs[0].PointsNum; j++)
                            {
                                if (j <= Vector3)
                                {
                                    ch1.Genes[j] = chs[pairs[2 * i]].Genes[j];
                                    ch2.Genes[j] = chs[pairs[2 * i + 1]].Genes[j];
                                }
                                else
                                {
                                    G1 = chs[pairs[2 * i]].Genes[j];
                                    G2 = chs[pairs[2 * i + 1]].Genes[j];
                                    ch1.Genes[j].Item1 = (G1.Item1 * W1 + G2.Item1 * W2) / (W1 + W2);
                                    ch1.Genes[j].Item2 = G1.Item2;
                                    ch2.Genes[j].Item1 = (G1.Item1 * W2 + G2.Item1 * W1) / (W1 + W2);
                                    ch2.Genes[j].Item2 = G2.Item2;
                                }
                            }

                            crossed[2 * i] = ch1;
                            crossed[2 * i + 1] = ch2;
                        }
                        else
                        {
                            crossed[2 * i] = chs[pairs[2 * i]];
                            crossed[2 * i + 1] = chs[pairs[2 * i + 1]];
                        }
                    }
                    break;
            }

            crossed.CopyTo(chs, 0);
        }
        #endregion
    }
}