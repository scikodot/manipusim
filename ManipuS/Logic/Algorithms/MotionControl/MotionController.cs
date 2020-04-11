using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Logic.PathPlanning;
using Logic.InverseKinematics;

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
        public ControllerState State = ControllerState.Idle;

        public MotionController(Obstacle[] obstacles, Manipulator agent, PathPlanner pathPlanner, IKSolver planSolver, IKSolver controlSolver, float deformThreshold)
        {
            Obstacles = obstacles;
            Agent = agent;
            PathPlanner = pathPlanner;
            PlanSolver = planSolver;
            ControlSolver = controlSolver;
            DeformThreshold = deformThreshold;
        }

        public void Execute(Vector3 goal)
        {
            State = ControllerState.Running;

            var res = PathPlanner.Execute(Obstacles, Agent, goal, PlanSolver);

            Agent.Path = res.Item1;
            Agent.States["Path"] = true;
            Agent.Configs = res.Item2;

            var contestant = Agent.DeepCopy();
            var jointPaths = new List<Vector3[]>();
            for (int i = 0; i < Agent.Path.Count; i++)
            {
                contestant.q = Agent.Configs[i];
                jointPaths.Add(contestant.DKP);
            }

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

            State = ControllerState.Finished;
        }

        public void Deform(Obstacle obstacle, Manipulator contestant, List<Vector3[]> jointPaths, int point, int joint)
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

        public List<Vector3[]> Discretize(Manipulator contestant, List<Vector3[]> jointPaths, int gripperPos)
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
    }
}
