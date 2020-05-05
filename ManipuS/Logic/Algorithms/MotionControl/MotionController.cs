using System.Collections.Generic;
using System.Numerics;
using Logic.PathPlanning;
using Logic.InverseKinematics;
using System.Diagnostics;
using System;
using System.Threading;

namespace Logic
{
    public enum ControllerState
    {
        Aborted = -1,  // TODO: can be used for exceptions processing
        Idle = 0,
        Running = 1,
        Finished = 2
    }

    public class MotionController
    {
        public Manipulator Agent;
        public Obstacle[] Obstacles;
        public PathPlanner PathPlanner;
        public IKSolver PlanSolver;
        public IKSolver ControlSolver;
        public float DeformThreshold;

        public ControllerState State { get; private set; } = ControllerState.Idle;
        public Stopwatch Timer { get; } = new Stopwatch();
        public Thread Thread { get; private set; }

        public MotionController(Obstacle[] obstacles, Manipulator agent, PathPlanner pathPlanner, IKSolver planSolver, IKSolver controlSolver, float deformThreshold)
        {
            Obstacles = obstacles;
            Agent = agent;
            PathPlanner = pathPlanner;
            PlanSolver = planSolver;
            ControlSolver = controlSolver;
            DeformThreshold = deformThreshold;
        }

        public void Run()
        {
            // update thread
            UpdateThread();

            // run thread
            Thread.Start();
        }

        public void Abort()
        {
            if (State == ControllerState.Running)
                Thread.Abort();
        }

        private void UpdateThread()
        {
            Thread = new Thread(() =>
            {
                try
                {
                    // start measuring execution time
                    Timer.Start();

                    // turn the controller on
                    State = ControllerState.Running;

                    // execute manipulator control process
                    ExecuteMotion(Agent.Goal);

                    // turn the controller off
                    State = ControllerState.Finished;
                }
                catch (ThreadAbortException e)  // checking for abort query
                {
                    // indicate that the process has been aborted
                    State = ControllerState.Aborted;
                }
                finally
                {
                    // stop measuring execution time
                    Timer.Reset();
                }
            });
        }

        private void ExecuteMotion(Vector3 goal)
        {
            // create attractors for the current manipulator
            CreateAttractors();

            // execute path planning
            var res = PathPlanner.Execute(Obstacles, Agent, goal, PlanSolver);

            // accumulate results
            Agent.Path = res.Item1;
            Agent.Configs = res.Item2;

            var contestant = Agent.DeepCopy();
            var jointPaths = new List<Vector3[]>();
            for (int i = 0; i < Agent.Path.Count; i++)
            {
                contestant.q = Agent.Configs[i];
                jointPaths.Add(contestant.DKP);
            }

            // execute motion control
            int gripperPos = Agent.posCounter;
            while (gripperPos < Agent.Configs.Count - 1)
            {
                // last point may not be deformed, since it is a goal point
                for (int i = gripperPos + 1; i < Agent.Path.Count - 1; i++)
                {
                    for (int j = jointPaths[i].Length - 1; j > 0; j--)
                    {
                        foreach (var obst in Obstacles)
                        {
                            if (obst.Contains(jointPaths[i][j]))
                            {
                                Deform(obst, contestant, jointPaths, i, j);
                                break;
                            }
                        }
                    }
                }

                jointPaths = Discretize(contestant, jointPaths, gripperPos);

                gripperPos = Agent.posCounter;
            }
        }

        private void Deform(Obstacle obstacle, Manipulator contestant, List<Vector3[]> jointPaths, int point, int joint)
        {
            Vector3 dx = obstacle.Extrude(jointPaths[point][joint]);

            Vector3 pNew = jointPaths[point][joint] + dx;
            contestant.q = Agent.Configs[point];
            Vector cNew = contestant.q + ControlSolver.Execute(Obstacles, contestant, pNew, joint).Item3;
            contestant.q = cNew;
            Vector3[] dkpNew = contestant.DKP;

            Agent.Path[point] = dkpNew[Agent.Joints.Length - 1];
            jointPaths[point] = dkpNew;
            Agent.Configs[point] = cNew;
        }

        private List<Vector3[]> Discretize(Manipulator contestant, List<Vector3[]> jointPaths, int gripperPos)
        {
            var discretizedPath = Agent.Path.GetRange(0, gripperPos + 1);
            var discretizedPaths = jointPaths.GetRange(0, gripperPos + 1);
            var discretizedConfigs = Agent.Configs.GetRange(0, gripperPos + 1);

            for (int i = gripperPos + 1; i < Agent.Path.Count; i++)
            {
                if (Agent.Path[i].DistanceTo(Agent.Path[i - 1]) > DeformThreshold)
                {
                    Vector3 pPrev = (Agent.Path[i] + Agent.Path[i - 1]) / 2;
                    contestant.q = Agent.Configs[i];
                    Vector cPrev = contestant.q + ControlSolver.Execute(Obstacles, contestant, pPrev, Agent.Joints.Length - 1).Item3;
                    contestant.q = cPrev;
                    Vector3[] dkpPrev = contestant.DKP;

                    discretizedPath.Add(dkpPrev[Agent.Joints.Length - 1]);
                    discretizedPaths.Add(dkpPrev);
                    discretizedConfigs.Add(cPrev);
                }

                discretizedPath.Add(jointPaths[i][Agent.Joints.Length - 1]);
                discretizedPaths.Add(jointPaths[i]);
                discretizedConfigs.Add(Agent.Configs[i]);
            }

            Agent.Path = discretizedPath;
            Agent.Configs = discretizedConfigs;

            return discretizedPaths;
        }

        public void CreateAttractors()
        {
            Agent.Attractors = new List<Attractor>();

            double workRadius = Agent.WorkspaceRadius;
            double x, yPos, y, zPos, z;

            // adding main attractor
            Vector3 attrPoint = Agent.Goal;
            float attrWeight = CalculateAttractorWeight(attrPoint);
            float attrRadius = CalculateAttractorRadius(attrWeight);

            Agent.Attractors.Add(new Attractor(attrPoint, attrWeight, attrRadius));

            // adding ancillary attractors
            while (Agent.Attractors.Count < WorkspaceBuffer.PathPlanningBuffer.AttrNum)
            {
                // generating attractor point
                x = RandomThreadStatic.NextDouble(workRadius);
                yPos = Math.Sqrt(workRadius * workRadius - x * x);
                y = RandomThreadStatic.NextDouble(yPos);
                zPos = Math.Sqrt(yPos * yPos - y * y);
                z = RandomThreadStatic.NextDouble(zPos);

                Vector3 point = new Vector3((float)x, (float)y, (float)z) + Agent.Base;

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
                    attrWeight = CalculateAttractorWeight(attrPoint);
                    attrRadius = CalculateAttractorRadius(attrWeight);

                    Agent.Attractors.Add(new Attractor(attrPoint, attrWeight, attrRadius));
                }
            }
        }

        private float CalculateAttractorWeight(Vector3 point)
        {
            return Agent.DistanceTo(point) + Agent.Goal.DistanceTo(point);
        }

        private float CalculateAttractorRadius(float weight)
        {
            return WorkspaceBuffer.PathPlanningBuffer.d * (float)Math.Pow(weight / Agent.DistanceTo(Agent.Goal), 4);
        }
    }
}
