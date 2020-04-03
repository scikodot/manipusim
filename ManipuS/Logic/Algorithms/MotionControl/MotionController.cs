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
        public Manipulator agent;
        public Obstacle[] Obstacles;
        public PathPlanner PathPlanner;
        public IKSolver Solver;

        public MotionController(Obstacle[] obstacles, Manipulator agent, PathPlanner pathPlanner, IKSolver solver)
        {
            Obstacles = obstacles;
            this.agent = agent;
            PathPlanner = pathPlanner;
            Solver = solver;
        }

        public void Execute(Vector3 goal)
        {
            var res = PathPlanner.Execute(Obstacles, agent, goal, Solver);

            agent.Path = res.Item1;
            agent.States["Path"] = true;
            agent.Configs = res.Item2;

            var contestant = agent.DeepCopy();
            List<List<Vector3>> paths = new List<List<Vector3>>();
            for (int i = 0; i < agent.Path.Count; i++)
            {
                contestant.q = agent.Configs[i];
                paths.Add(contestant.DKP.ToList());
            }

            var gripperPos = 0;
            while (gripperPos < agent.Configs.Count)
            {
                for (int i = gripperPos + 1; i < agent.Path.Count; i++)
                {
                    for (int j = paths[i].Count - 1; j > 0; j--)
                    {
                        foreach (var obst in Obstacles)
                        {
                            if (obst.Contains(paths[i][j]))
                            {
                                Deform(agent, obst, paths, i, j);
                                break;
                            }
                        }
                    }
                }
            }
        }

        public void Deform(Manipulator agent, Obstacle obstacle, List<List<Vector3>> paths, int point, int joint)
        {
            Vector3 vec = new Vector3(obstacle.Collider.Center, paths[point][joint]);
            Vector3 n = vec.Normalized;
            Vector3 dx = n * (obstacle.Collider as Sphere).Radius - vec;

            Vector3 pNew = paths[point][joint] + dx;
            var contestant = agent.DeepCopy();
            contestant.q = agent.Configs[point];

            Vector cNew = contestant.q + Solver.Execute(Obstacles, contestant, pNew, joint).Item3;
            contestant.q = cNew;
            List<Vector3> dkpNew = contestant.DKP.ToList();

            Vector3 pPrev = Vector3.Null, pNext = Vector3.Null;
            Vector cPrev = Vector.Null, cNext = Vector.Null;
            List<Vector3> dkpPrev = null, dkpNext = null;
            if (pNew.DistanceTo(paths[point - 1][joint]) >= 0.08)
            {
                pPrev = (pNew + paths[point - 1][joint]) / 2;
                contestant.q = agent.Configs[point];
                cPrev = contestant.q + Solver.Execute(Obstacles, contestant, pPrev, joint).Item3;
                contestant.q = cPrev;
                dkpPrev = contestant.DKP.ToList();
            }
            if (pNew.DistanceTo(paths[point + 1][joint]) >= 0.08)
            {
                pNext = (pNew + paths[point + 1][joint]) / 2;
                contestant.q = agent.Configs[point];
                cNext = contestant.q + Solver.Execute(Obstacles, contestant, pNext, joint).Item3;
                contestant.q = cNext;
                dkpNext = contestant.DKP.ToList();
            }

            if (pPrev != Vector3.Null)
            {
                paths.Insert(point, dkpPrev);
                agent.Path.Insert(point, dkpPrev[agent.Joints.Length]);  // TODO: optimize; insertions are always costly
                agent.Configs.Insert(point++, cPrev);
            }

            paths[point] = dkpNew;
            agent.Path[point] = dkpNew[agent.Joints.Length];
            agent.Configs[point] = cNew;

            if (pNext != Vector3.Null)
            {
                agent.Path.Insert(++point, dkpNext[agent.Joints.Length]);
                agent.Configs.Insert(point, cNext);
                paths.Insert(point, dkpNext);
            }
        }
    }
}
