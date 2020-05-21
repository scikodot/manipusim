using Graphics;
using Logic;
using Logic.InverseKinematics;
using Logic.PathPlanning;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

public static class Benchmark
{
    private static Stopwatch _timer = new Stopwatch();

    private static int _samplesIK = 10;
    private static int _maxTimeIK = 100;
    private static int _samplesPP = 10;
    private static int _maxTimePP = 10000;

    // inverse kinematics planners
    private static InverseKinematicsSolver[] _solvers =
    {
        new JacobianTranspose(0.05f, 0, _maxTimeIK),
        new JacobianPseudoinverse(0.05f, 0, _maxTimeIK),
        new DampedLeastSquares(0.05f, 0, _maxTimeIK),
        new HillClimbing(0.05f, 3, 10 * _maxTimeIK)
    };

    private static PathPlanner[] _planners =
    {
        new RRT(_maxTimePP, true, 0.04f),
        new DynamicRRT(_maxTimePP, true, 0.04f, 0)
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
                    if (i > 1)
                        stream.WriteLine($"{res.Item2},{_timer.ElapsedTicks / 10},{res.Item3}");
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
        CreateAttractors(agent);

        foreach (var planner in _planners)
        {
            var type = planner.GetType().Name;
            using (var stream = new StreamWriter($@"D:\ManipusBenchmark\Planners\{type}.csv"))
            {
                stream.WriteLine("Iterations,Time,PathLength,Error");

                for (int i = 0; i < _samplesPP + 2; i++)
                {
                    _timer.Restart();
                    var path = planner.Execute(ObstacleHandler.Obstacles.ToArray(), agent, agent.Goal, new DampedLeastSquares(0.05f, 0, _maxTimeIK));
                    _timer.Stop();

                    // discard first few results because Stopwatch has a warmup phase which produces excessively high numbers
                    if (i > 1)
                        stream.WriteLine($"{planner.Iterations - 1},{_timer.ElapsedTicks / 10},{path.Count},{path.Last.Points[path.Last.Points.Length - 1].DistanceTo(agent.Goal)}");
                }
            }
        }
    }

    public static void CreateAttractors(Manipulator agent)  // TODO: move to Attractor class
    {
        agent.Attractors = new List<Attractor>();

        double workRadius = agent.WorkspaceRadius;
        double x, yPos, y, zPos, z;

        // adding main attractor
        Vector3 attrPoint = agent.Goal;
        float attrWeight = CalculateAttractorWeight(agent, attrPoint);
        float attrRadius = CalculateAttractorRadius(agent, attrWeight);

        agent.Attractors.Add(new Attractor(attrPoint, attrWeight, attrRadius));

        // adding ancillary attractors
        while (agent.Attractors.Count < WorkspaceBuffer.PathPlanningBuffer.AttrNum)
        {
            // generating attractor point
            x = RandomThreadStatic.NextDouble(workRadius);
            yPos = Math.Sqrt(workRadius * workRadius - x * x);
            y = RandomThreadStatic.NextDouble(yPos);
            zPos = Math.Sqrt(yPos * yPos - y * y);
            z = RandomThreadStatic.NextDouble(zPos);

            Vector3 point = new Vector3((float)x, (float)y, (float)z) + agent.Base;

            // checking whether the attractor is inside any obstacle or not
            bool collision = false;
            foreach (var obst in ObstacleHandler.Obstacles)
            {
                if (obst.Contains(point))
                {
                    collision = true;
                    break;
                }
            }

            if (!collision)  // TODO: consider creating a list of bad attractors; they may serve as repulsion points
            {
                // adding attractor to the list
                attrPoint = point;
                attrWeight = CalculateAttractorWeight(agent, attrPoint);
                attrRadius = CalculateAttractorRadius(agent, attrWeight);

                agent.Attractors.Add(new Attractor(attrPoint, attrWeight, attrRadius));
            }
        }
    }

    private static float CalculateAttractorWeight(Manipulator agent, Vector3 point)
    {
        return agent.DistanceTo(point) + agent.Goal.DistanceTo(point);
    }

    private static float CalculateAttractorRadius(Manipulator agent, float weight)
    {
        return WorkspaceBuffer.PathPlanningBuffer.d * (float)Math.Pow(weight / agent.DistanceTo(agent.Goal), 4);
    }
}
