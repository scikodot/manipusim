using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

using MoreLinq;

using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    [Flags]
    public enum OptimizationCriteria  // TODO: add more criteria?
    {
        NoOptimization = 0,
        CollisionFree = 1,
        PathLength = 2
    }

    public class Chromosome
    {
        public BezierCurve BezierCurve { get; }
        public Path Path { get; }
        public float Weight { get; }

        public Chromosome(BezierCurve bezierCurve, Path path, float weight)
        {
            BezierCurve = bezierCurve;
            Path = path;
            Weight = weight;
        }
    }

    public class GeneticAlgorithm : PathPlanner
    {
        protected static int _offspringSizeDefault = 4;
        protected static int _survivalSizeDefault = 10;
        protected static int _bezierControlPointsCountDefault = 2;
        protected static float _bezierStepDefault = 0.01f;

        private OptimizationCriteria _optimizationCriteria = OptimizationCriteria.CollisionFree;  // TODO: add to GUI
        public ref OptimizationCriteria OptimizationCriteria => ref _optimizationCriteria;

        private Manipulator _manipulator;
        private VectorFloat _initialConfiguration;
        private InverseKinematicsSolver _solver;
        private Vector3 _goal;

        public static volatile Chromosome Dominant;
        public static volatile bool Locked;
        public static volatile bool Changed;

        private int _offspringSize;
        public ref int OffspringSize => ref _offspringSize;

        private int _survivalSize;
        public ref int SurvivalSize => ref _survivalSize;

        private int _bezierControlPointsCount;
        public ref int BezierControlPointsCount => ref _bezierControlPointsCount;

        private float _bezierStep;
        public ref float BezierStep => ref _bezierStep;

        public GeneticAlgorithm(int maxIterations, float threshold, bool collisionCheck, 
            int offspringSize, int survivalSize, int bezierControlPointsCount, float bezierStep) : base(maxIterations, threshold, collisionCheck)
        {
            _offspringSize = offspringSize;
            _survivalSize = survivalSize;
            _bezierControlPointsCount = bezierControlPointsCount;
            _bezierStep = bezierStep;
        }

        public static GeneticAlgorithm Default()
        {
            return new GeneticAlgorithm(_maxIterationsDefault / 100, _thresholdDefault, _collisionCheckDefault, 
                _offspringSizeDefault, _survivalSizeDefault, _bezierControlPointsCountDefault, _bezierStepDefault);
        }

        protected override PathPlanningResult RunAbstract(Manipulator manipulator, Vector3 goal, InverseKinematicsSolver solver)
        {
            _manipulator = manipulator;
            _initialConfiguration = manipulator.q;
            _solver = solver;
            _goal = goal;

            int generationsCount = 0;
            var generation = new List<Chromosome>();

            // get initial solution
            var bezierInitial = ConstructBezier(_manipulator.GripperPos, goal, _bezierControlPointsCount);
            var pathInitial = ConstructPath(bezierInitial, _bezierStep);
            var chromosomeInitial = new Chromosome(bezierInitial, pathInitial, Rate(pathInitial));

            generation.Add(chromosomeInitial);

            Dominant = chromosomeInitial;

            // evolve generations
            Console.SetCursorPosition(0, 10);
            Console.WriteLine("Starting genetic algorithm...");
            while (generation[0].Weight > 0 && generationsCount++ < _maxIterations)
            {
                // get new generation
                generation = Evolve(generation, _offspringSize, _survivalSize);

                Console.SetCursorPosition(0, 11);
                Console.WriteLine($"Generations passed: {generationsCount}");
                Console.WriteLine($"Best fit: {generation[0].Weight}");

                if (!Locked)
                {
                    Dominant = generation[0];
                    Changed = true;
                }
            }

            Console.SetCursorPosition(0, 15);
            Console.WriteLine("Found path's Bezier curve:");
            foreach (var point in generation[0].BezierCurve.Points)
            {
                Console.WriteLine(point);
            }

            return new PathPlanningResult
            {
                Iterations = generationsCount - 1,
                Path = generation[0].Path
            };
        }

        private BezierCurve ConstructBezier(Vector3 start, Vector3 end, int intermediaryPoints)
        {
            Vector3[] points = new Vector3[2 + intermediaryPoints];

            Vector3 direction = end - start;
            Vector3 directionNorm = Vector3.Normalize(direction);
            float length = direction.Length();
            float segmentCountInv = 1.0f / (points.Length - 1);

            points[0] = start;
            for (int i = 1; i < points.Length - 1; i++)
            {
                points[i] = start + directionNorm * i * length * segmentCountInv;
            }
            points[points.Length - 1] = end;

            return new BezierCurve(points);
        }

        private Path ConstructPath(BezierCurve bezierCurve, float step)  // TODO: refactor!
        {
            // reset agent
            _manipulator.q = _initialConfiguration;

            float counter = 0;

            var points = new List<Vector3[]> { _manipulator.DKP };
            var configs = new List<VectorFloat> { _manipulator.q };
            while (counter <= 1)
            {
                var ikRes = _solver.Execute(_manipulator, bezierCurve.CalculatePoint(counter));

                _manipulator.q = ikRes.Configuration;

                points.Add(_manipulator.DKP);
                configs.Add(_manipulator.q);

                counter += step;
            }

            return new Path(points, configs);
        }

        private List<Chromosome> Evolve(List<Chromosome> generation, int offspringSize, int survivalSize)
        {
            var offsprings = new List<Chromosome>();
            foreach (var member in generation)
            {
                offsprings.AddRange(Reproduce(member, offspringSize));
            }

            return Select(offsprings, survivalSize);
        }

        private List<Chromosome> Reproduce(Chromosome member, int offspringSize)
        {
            var repro = new List<Chromosome>();
            for (int i = 0; i < offspringSize; i++)
            {
                BezierCurve bezierCurveRepro = Repro(member.BezierCurve);
                Path pathRepro = ConstructPath(bezierCurveRepro, _bezierStep);
                repro.Add(new Chromosome(bezierCurveRepro, pathRepro, Rate(pathRepro)));
            }

            return repro;
        }

        private BezierCurve Repro(BezierCurve bezierCurve)
        {
            int pointIndex = RandomThreadStatic.Next(1, _bezierControlPointsCount + 1);
            Vector3 direction = RandomThreadStatic.NextPointCube(1);

            var bezierCurveRepro = new BezierCurve(bezierCurve.Points);
            var bezierDirectionRepro = Vector3.Normalize(direction) * 1f * (float)RandomThreadStatic.NextDouble();

            bezierCurveRepro.Points[pointIndex] += bezierDirectionRepro;

            if (_manipulator.ApproxWithinReach(bezierCurveRepro.Points[pointIndex] + bezierDirectionRepro))
                bezierCurveRepro.Points[pointIndex] += bezierDirectionRepro;
            else
                bezierCurveRepro.Points[pointIndex] -= bezierDirectionRepro;

            return bezierCurveRepro;
        }

        private List<Chromosome> Select(List<Chromosome> offsprings, int survivalSize)
        {
            return offsprings.OrderBy(x => x.Weight).Take(survivalSize).ToList();
        }

        private float Rate(Path sample)
        {
            float score = 0;

            // apply goal convergence criterion
            _manipulator.q = sample.Last.q;
            var distance = _manipulator.DistanceTo(_goal);
            if (distance > _threshold)
                score += distance - _threshold;

            // extract parameters' values from chromosome
            foreach (var node in sample.Nodes)
            {
                _manipulator.q = node.q;

                Vector3 currPos = node.Points[node.Points.Length - 1];

                // apply fitness functions to the given chromosome's point
                int criteriaCount = 0;
                float pointWeight = 0;

                if (_optimizationCriteria.HasFlag(OptimizationCriteria.CollisionFree))
                {
                    if (ObstacleHandler.ContainmentTest(currPos, out _))
                    {
                        pointWeight += 1;
                    }

                    if (_manipulator.CollisionTest().Contains(true))
                    {
                        pointWeight += 1;
                    }

                    criteriaCount++;
                }
                if (_optimizationCriteria.HasFlag(OptimizationCriteria.PathLength))
                {

                }

                // take median of all criteria weights
                score += pointWeight / criteriaCount;
            }

            return score;
        }
    }
}