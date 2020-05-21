using Graphics;
using Logic;
using Logic.InverseKinematics;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

public static class Benchmark
{
    private static Stopwatch _timer = new Stopwatch();

    private static int _samplesIK = 5;
    private static int _maxTimeIK = 100;
    private static int _samplesPP = 50;

    // inverse kinematics planners
    private static InverseKinematicsSolver[] _solvers =
    {
        new JacobianTranspose(0.05f, 0, _maxTimeIK),
        new JacobianInverse(0.05f, 0, _maxTimeIK),
        new DampedLeastSquares(0.05f, 0, _maxTimeIK),
        new HillClimbing(0.05f, 3, 5 * _maxTimeIK)
    };

    //private static InverseKinematicsSolver _jacobianTranspose = new JacobianTranspose(0.05f, 0, _maxTimeIK);
    //private static InverseKinematicsSolver _jacobianInverse = new JacobianInverse(0.05f, 0, _maxTimeIK);
    //private static InverseKinematicsSolver _dampedLeastSquares = new DampedLeastSquares(0.05f, 0, _maxTimeIK);
    //private static InverseKinematicsSolver _hillClimbing = new HillClimbing(0.05f, 2, _maxTimeIK);

    public static void RunInverseKinematics()
    {
        var agent = ManipulatorHandler.Manipulators[0];
        var agentCopy = agent.DeepCopy();
        foreach (var solver in _solvers)
        {
            var type = solver.GetType().Name;
            using (var stream = new StreamWriter($@"D:\ManipusBenchmark\Solvers\{type}.txt"))
            {
                stream.WriteLine("Time, Error");

                for (int i = 0; i < _samplesIK; i++)
                {
                    agentCopy.q = agent.q;

                    _timer.Restart();
                    var res = solver.Execute(agentCopy, agentCopy.Goal, agentCopy.Joints.Length - 1);
                    _timer.Stop();

                    stream.WriteLine($"{_timer.ElapsedTicks / 10.0f}, {res.Item2}");
                }
            }
        }
    }

    public static void RunPathPlanning()
    {

    }
}
