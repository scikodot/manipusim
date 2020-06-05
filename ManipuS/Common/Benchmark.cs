using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

using Logic;
using Logic.InverseKinematics;
using Logic.PathPlanning;
using MoreLinq;

public static class Benchmark
{
    private static Stopwatch _timer = new Stopwatch();

    private static bool _isErrorTaken;
    private static int _samplesIK = 100;
    private static int _samplesPP = 10;

    // inverse kinematics planners
    private static List<InverseKinematicsSolver> _solvers = new List<InverseKinematicsSolver>
    {
        JacobianTranspose.Default(),
        JacobianPseudoinverse.Default(),
        DampedLeastSquares.Default(),
        HillClimbing.Default()
    };

    private static List<PathPlanner> _planners = new List<PathPlanner>
    {
        RRT.Default(),
        GeneticAlgorithm.Default()        
    };

    private static DampedLeastSquares _plannerSolver = DampedLeastSquares.Default();

    public static void RunInverseKinematics()
    {
        _isErrorTaken = false;

        // setup dot separator for floats
        System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
        customCulture.NumberFormat.NumberDecimalSeparator = ".";

        System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;

        var agent = ManipulatorHandler.Manipulators[0];
        foreach (var solver in _solvers)
        {
            var type = solver.GetType().Name;

            using (var stream = new StreamWriter($@"C:\Users\Dan\PyScripts\ManipusBenchmark\Solvers\{type}.csv"))
            {
                stream.WriteLine("Iterations,Time,Error");

                for (int i = 0; i < _samplesIK + 2; i++)
                {
                    _timer.Restart();
                    var res = solver.Execute(agent, agent.Goal, agent.Joints.Length - 1);
                    _timer.Stop();

                    // discard first few results because Stopwatch has a warmup phase which produces excessively high numbers
                    if (i > 1)
                        stream.WriteLine($"{res.Iterations},{_timer.Elapsed.TotalMilliseconds},{res.Error.L2Norm()}");
                }
            }

            using (var stream = new StreamWriter($@"C:\Users\Dan\PyScripts\ManipusBenchmark\Solvers\{type}_Error.csv"))
            {
                solver.errorMod = new List<double>();
                solver.configs = new List<MathNet.Numerics.LinearAlgebra.Vector<float>>();

                _timer.Restart();
                var res = solver.Execute(agent, agent.Goal, agent.Joints.Length - 1);
                _timer.Stop();

                string header = "Time,";
                for (int i = 0; i < solver.configs[0].Count; i++)
                {
                    header += $"Joint{i},";
                }

                header += "Error";

                stream.WriteLine(header);

                for (int i = 0; i < solver.errorMod.Count; i++)
                {
                    string str = $"{i * _timer.Elapsed.TotalMilliseconds / solver.errorMod.Count},";
                    for (int j = 0; j < solver.configs[i].Count; j++)
                    {
                        str += solver.configs[i][j] + ",";
                    }

                    str += solver.errorMod[i];

                    stream.WriteLine(str);
                }
            }
        }
    }

    public static void RunPathPlanning()
    {
        // setup dot separator for floats
        System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
        customCulture.NumberFormat.NumberDecimalSeparator = ".";

        System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;

        var agent = ManipulatorHandler.Manipulators[0];
        _planners.Add(ARRT.Default(agent));

        foreach (var planner in _planners)
        {
            var type = planner.GetType().Name;
            using (var stream = new StreamWriter($@"C:\Users\Dan\PyScripts\ManipusBenchmark\Planners\{type}.csv"))
            {
                stream.WriteLine("Iterations,Time,PathLength,Error");

                for (int i = 0; i < _samplesPP + 1; i++)
                {
                    PathPlanningResult ppRes;
                    using (var manipulatorCopy = agent.DeepCopy())
                    {
                        _timer.Restart();
                        ppRes = planner.Run(manipulatorCopy, manipulatorCopy.Goal, _plannerSolver);
                        _timer.Stop();
                    }

                    // discard first few results because Stopwatch has a warmup phase which produces excessively high numbers
                    if (i > 0)
                        stream.WriteLine($"{ppRes.Iterations},{_timer.Elapsed.TotalMilliseconds},{ppRes.Path.GetLength()}," +
                            $"{ppRes.Path.Last.Points[ppRes.Path.Last.Points.Length - 1].DistanceTo(agent.Goal)}");
                }
            }
        }
    }
}
