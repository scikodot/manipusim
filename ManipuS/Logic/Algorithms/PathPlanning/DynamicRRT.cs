using System;
using System.Collections.Generic;
using System.Linq;
using Logic.InverseKinematics;
using System.Threading;
using System.Threading.Tasks;

namespace Logic.PathPlanning
{
    class DynamicRRT : PathPlanner
    {
        private float d;
        private int period;

        public DynamicRRT(Obstacle[] obstacles, IKSolver solver, int maxTime, bool collisionCheck, float d, int period) : base(obstacles, solver, maxTime, collisionCheck)
        {
            this.d = d;
            this.period = period;
        }

        public void Start(Manipulator agent, Vector3 goal)
        {
            var res = Execute(agent, goal);

            agent.Path = res.Item1;
            agent.States["Path"] = true;
            agent.Configs = res.Item2;

            var contestant = new Manipulator(agent);
            List<List<Vector3>> paths = new List<List<Vector3>>();
            for (int i = 0; i < agent.Path.Count; i++)
            {
                contestant.q = Misc.CopyArray(agent.Configs[i]);
                paths.Add(contestant.DKP.ToList());
            }

            var control = Task.Run(() => Control(agent, goal, paths));
        }

        public void Control(Manipulator agent, Vector3 goal, List<List<Vector3>> paths)
        {
            var contestant = new Manipulator(agent);

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

                // wait for window to draw current config so that we can update it
                Dispatcher.UpdateConfig.WaitOne();
                agent.q = agent.Configs[gripperPos < agent.Configs.Count - 1 ? gripperPos++ : gripperPos];
                Dispatcher.UpdateConfig.Reset();
            }
        }

        public void Deform(Manipulator agent, Obstacle obstacle, List<List<Vector3>> paths, int point, int joint)
        {
            Vector3 vec = new Vector3(obstacle.Collider.Center, paths[point][joint]);
            Vector3 n = vec.Normalized;
            Vector3 dx = n * (obstacle.Collider as Sphere).Radius - vec;

            Vector3 pNew = paths[point][joint] + dx;
            var contestant = new Manipulator(agent)
            {
                q = Misc.CopyArray(agent.Configs[point])
            };
            float[] cNew = contestant.q.Zip(Solver.Execute(contestant, pNew, joint).Item3, (t, s) => t + s).ToArray();
            contestant.q = Misc.CopyArray(cNew);
            List<Vector3> dkpNew = contestant.DKP.ToList();

            Vector3 pPrev = Vector3.Null, pNext = Vector3.Null;
            float[] cPrev = null, cNext = null;
            List<Vector3> dkpPrev = null, dkpNext = null;
            if (pNew.DistanceTo(paths[point - 1][joint]) >= 2 * d)
            {
                pPrev = (pNew + paths[point - 1][joint]) / 2;
                contestant.q = Misc.CopyArray(agent.Configs[point]);
                cPrev = contestant.q.Zip(Solver.Execute(contestant, pPrev, joint).Item3, (t, s) => t + s).ToArray();
                contestant.q = Misc.CopyArray(cPrev);
                dkpPrev = contestant.DKP.ToList();
            }
            if (pNew.DistanceTo(paths[point + 1][joint]) >= 2 * d)
            {
                pNext = (pNew + paths[point + 1][joint]) / 2;
                contestant.q = Misc.CopyArray(agent.Configs[point]);
                cNext = contestant.q.Zip(Solver.Execute(contestant, pNext, joint).Item3, (t, s) => t + s).ToArray();
                contestant.q = Misc.CopyArray(cNext);
                dkpNext = contestant.DKP.ToList();
            }

            if (pPrev != Vector3.Null)
            {
                paths.Insert(point, dkpPrev);
                agent.Path.Insert(point, dkpPrev[agent.Joints.Length]);
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

        public override (List<Vector3>, List<float[]>) Execute(Manipulator agent, Vector3 goal)
        {
            Manipulator Contestant = new Manipulator(agent);

            // creating new tree
            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));

            // sorting attractors for easier work
            var Attractors = new List<Attractor>(agent.GoodAttractors);
            Attractors.Sort((t, s) => { return t.Weight <= s.Weight ? (t.Weight < s.Weight ? -1 : 0) : 1; });

            for (int i = 0; i < MaxTime; i++)
            {
                if (i % period == 0 && i != 0)
                    Trim(agent.Tree, Contestant);

                // generating normally distributed value with Box-Muller transform
                float num = Misc.BoxMullerTransform(Rng, Attractors[0].Weight, (Attractors[Attractors.Count - 1].Weight - Attractors[0].Weight) / 3);  // TODO: check distribution!

                // extracting the first relevant attractor
                Attractor attr = Attractors.Find((t) => { return t.Weight > num; });

                int index = 0;
                if (attr == null)  // clamping weight
                    index = Attractors.Count - 1;
                else
                    index = Attractors.IndexOf(attr);

                float radius = Attractors[index].Radius, x, y_pos, y, z_pos, z;

                // generating Vector3 of attraction (inside the attractor's field) for tree
                x = -radius + (float)Rng.NextDouble() * 2 * radius;
                y_pos = (float)Math.Sqrt(radius * radius - x * x);
                y = -y_pos + (float)Rng.NextDouble() * 2 * y_pos;
                z_pos = (float)Math.Sqrt(radius * radius - x * x - y * y);
                z = -z_pos + (float)Rng.NextDouble() * 2 * z_pos;

                Vector3 p = new Vector3(x, y, z) + Attractors[index].Center;

                // finding the closest node to the generated Vector3
                Tree.Node minNode = agent.Tree.Min(p);

                // creating offset vector to new node
                Vector3 v = new Vector3(minNode.p, p);
                Vector3 pNew = minNode.p + v.Normalized * d;

                // checking for collisions of the new node
                bool collision = false;
                if (CollisionCheck)
                {
                    foreach (var obst in Obstacles)
                    {
                        if (obst.Contains(pNew))
                        {
                            collision = true;
                            break;
                        }
                    }
                }

                if (!collision)
                {
                    // solving IKP for new node
                    Contestant.q = Misc.CopyArray(minNode.q);
                    var res = Solver.Execute(Contestant, pNew, Contestant.Joints.Length);
                    var pos = Contestant.GripperPos;
                    if (res.Item1 && !(CollisionCheck && res.Item4.Contains(true)))
                    {
                        // adding node to the tree
                        Tree.Node node = new Tree.Node(minNode, Contestant.GripperPos, Contestant.q);
                        agent.Tree.AddNode(node);
                        if (pNew.DistanceTo(Attractors[index].Center) < Attractors[index].Radius)
                        {
                            // removing attractor if it has been hit
                            if (index != 0)
                                Attractors.RemoveAt(index);
                            else
                                Attractors[index].InliersCount++;
                        }
                    }
                }

                // stopping in case the main attractor has been hit
                if (Attractors[0].InliersCount != 0)
                    break;
            }

            // retrieving resultant path along with respective configurations
            Tree.Node start = agent.Tree.Min(agent.Goal), node_curr = start;
            List<Vector3> path = new List<Vector3>();
            List<float[]> configs = new List<float[]>();
            for (int i = start.Layer; i >= 0; i--)
            {
                if (node_curr.Layer == i)
                {
                    path.Add(node_curr.p);
                    configs.Add(node_curr.q);
                    if (node_curr.Parent != null)
                    {
                        int Vector3sNum = node_curr.Layer - node_curr.Parent.Layer - 1;
                        if (Vector3sNum > 0)
                        {
                            Tree.Node[] nodes = Tree.Discretize(node_curr, node_curr.Parent, Vector3sNum);
                            foreach (var node in nodes)
                            {
                                configs.Add(node.q);
                            }
                        }
                    }

                    node_curr = node_curr.Parent;
                }
            }

            // reverting path so that it goes from root to goal
            path.Reverse();
            configs.Reverse();

            return (path, configs);
        }

        private void Trim(Tree tree, Manipulator contestant)
        {
            for (int i = tree.Layers.Count - 1; i > 0; i--)
            {
                for (int j = tree.Layers[i].Count - 1; j >= 0; j--)
                {
                    // check node Vector3 for collisions
                    bool nodeRemoved = false;
                    foreach (var obst in Obstacles)
                    {
                        if (obst.Contains(tree.Layers[i][j].p))
                        {
                            tree.Layers[i][j].Parent.Childs.Remove(tree.Layers[i][j]);
                            tree.RemoveNode(tree.Layers[i][j]);
                            nodeRemoved = true;
                            break;
                        }
                    }
                    if (nodeRemoved)
                        continue;

                    // check node config for collisions
                    contestant.q = tree.Layers[i][j].q;
                    if (Solver.DetectCollisions(contestant, Obstacles).Contains(true))
                    {
                        tree.Layers[i][j].Parent.Childs.Remove(tree.Layers[i][j]);
                        tree.RemoveNode(tree.Layers[i][j]);
                    }
                }
            }
        }
    }
}
