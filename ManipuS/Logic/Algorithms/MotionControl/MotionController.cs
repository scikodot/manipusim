using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Logic.PathPlanning;
using Logic.InverseKinematics;

namespace Logic
{
    public class MotionController
    {
        public Manipulator Agent;
        public Obstacle[] Obstacles;
        public PathPlanner PathPlanner;
        public IKSolver Solver;

        public MotionController(Obstacle[] obstacles, Manipulator agent, PathPlanner pathPlanner, IKSolver solver)
        {
            Obstacles = obstacles;
            Agent = agent;
            PathPlanner = pathPlanner;
            Solver = solver;
        }

        public void Execute(Vector3 goal)
        {
            var res = PathPlanner.Execute(Obstacles, Agent, goal, Solver);

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
                for (int i = gripperPos + 1; i < Agent.Path.Count; i++)
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

        public void Deform(Obstacle obstacle, Manipulator contestant, List<Vector3[]> jointPaths, int point, int joint)
        {
            Vector3 vec = jointPaths[point][joint] - obstacle.Collider.Center;
            Vector3 dx = vec.Normalized * (obstacle.Collider as Sphere).Radius - vec;

            Vector3 pNew = jointPaths[point][joint] + dx;
            contestant.q = Agent.Configs[point];
            Vector cNew = contestant.q + Solver.Execute(Obstacles, contestant, pNew, joint).Item3;
            contestant.q = cNew;
            Vector3[] dkpNew = contestant.DKP;

            Agent.Path[point] = dkpNew[Agent.Joints.Length];
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
                if (Agent.Path[i].DistanceTo(Agent.Path[i - 1]) > 0.08)
                {
                    Vector3 pPrev = (Agent.Path[i] + Agent.Path[i - 1]) / 2;
                    contestant.q = Agent.Configs[i];
                    Vector cPrev = contestant.q + Solver.Execute(Obstacles, contestant, pPrev, Agent.Joints.Length).Item3;
                    contestant.q = cPrev;
                    Vector3[] dkpPrev = contestant.DKP;

                    discretizedPath.Add(dkpPrev[Agent.Joints.Length]);
                    discretizedPaths.Add(dkpPrev);
                    discretizedConfigs.Add(cPrev);
                }

                discretizedPath.Add(jointPaths[i][Agent.Joints.Length]);
                discretizedPaths.Add(jointPaths[i]);
                discretizedConfigs.Add(Agent.Configs[i]);
            }

            Agent.Path = discretizedPath;
            Agent.Configs = discretizedConfigs;

            return discretizedPaths;
        }
    }
}
