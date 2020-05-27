using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

using Logic;
using Logic.InverseKinematics;
using Logic.PathPlanning;

public static class Benchmark
{
    private static Stopwatch _timer = new Stopwatch();

    private static int _samplesIK = 10;
    private static int _samplesPP = 10;

    // inverse kinematics planners
    private static List<InverseKinematicsSolver> _solvers = new List<InverseKinematicsSolver>
    {
        //JacobianTranspose.Default(),
        //JacobianPseudoinverse.Default(),
        //DampedLeastSquares.Default(),
        //HillClimbing.Default()
    };

    private static List<PathPlanner> _planners = new List<PathPlanner>
    {
        RRT.Default()
    };

    public static void RunInverseKinematics()
    {
        // setup dot separator for floats
        System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
        customCulture.NumberFormat.NumberDecimalSeparator = ".";

        System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;

        var agent = ManipulatorHandler.Manipulators[0];
        var agentCopy = agent.DeepCopy();
        foreach (var solver in _solvers)
        {
            var type = solver.GetType().Name;
            using (var stream = new StreamWriter($@"D:\ManipusBenchmark\Solvers\{type}.csv"))
            {
                stream.WriteLine("Iterations,Time,Error");

                for (int i = 0; i < _samplesIK + 2; i++)
                {
                    agentCopy.q = agent.q;

                    _timer.Restart();
                    var res = solver.Execute(agentCopy, agentCopy.Goal, agentCopy.Joints.Length - 1);
                    _timer.Stop();

                    // discard first few results because Stopwatch has a warmup phase which produces excessively high numbers
                    //if (i > 1)
                    //    stream.WriteLine($"{res.Item2},{_timer.ElapsedTicks / 10},{res.Item3}");
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
            using (var stream = new StreamWriter($@"D:\ManipusBenchmark\Planners\{type}.csv"))
            {
                stream.WriteLine("Iterations,Time,PathLength,Error");

                for (int i = 0; i < _samplesPP + 2; i++)
                {
                    _timer.Restart();
                    var ppRes = planner.Run(agent, agent.Goal, DampedLeastSquares.Default());
                    _timer.Stop();

                    // discard first few results because Stopwatch has a warmup phase which produces excessively high numbers
                    if (i > 1)
                        stream.WriteLine($"{ppRes.Iterations},{_timer.ElapsedTicks / 10},{ppRes.Path.Count}," +
                            $"{ppRes.Path.Last.Points[ppRes.Path.Last.Points.Length - 1].DistanceTo(agent.Goal)}");
                }
            }
        }
    }
}
