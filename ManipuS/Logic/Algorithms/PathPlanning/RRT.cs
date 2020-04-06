//using System;
//using System.Collections.Generic;
//using System.Linq;
//using Logic.InverseKinematics;

//namespace Logic.PathPlanning
//{
//    class RRT : PathPlanner
//    {
//        private float d;

//        public RRT(int maxTime, bool collisionCheck, float d) : base(maxTime, collisionCheck)
//        {
//            this.d = d;
//        }

//        public override (List<Vector3>, List<Vector>) Execute(Obstacle[] Obstacles, Manipulator agent, Vector3 goal, IKSolver Solver)
//        {
//            var Contestant = agent.DeepCopy();

//            // creating new tree
//            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));

//            // sorting attractors for easier work
//            var Attractors = new List<Attractor>(agent.GoodAttractors);
//            Attractors.Sort((t, s) => { return t.Weight <= s.Weight ? (t.Weight < s.Weight ? -1 : 0) : 1; });

//            for (int i = 0; i < MaxTime; i++)
//            {
//                // generating normally distributed value with Box-Muller transform
//                float num = Misc.BoxMullerTransform(Rng, Attractors[0].Weight, (Attractors[Attractors.Count - 1].Weight - Attractors[0].Weight) / 3);  // TODO: check distribution!

//                // extracting the first relevant attractor
//                Attractor attr = Attractors.Find((t) => { return t.Weight > num; });

//                int index = 0;
//                if (attr == null)  // clamping weight
//                    index = Attractors.Count - 1;
//                else
//                    index = Attractors.IndexOf(attr);

//                float radius = Attractors[index].Radius, x, y_pos, y, z_pos, z;

//                // generating Vector3 of attraction (inside the attractor's field) for tree
//                x = -radius + (float)Rng.NextDouble() * 2 * radius;
//                y_pos = (float)Math.Sqrt(radius * radius - x * x);
//                y = -y_pos + (float)Rng.NextDouble() * 2 * y_pos;
//                z_pos = (float)Math.Sqrt(radius * radius - x * x - y * y);
//                z = -z_pos + (float)Rng.NextDouble() * 2 * z_pos;

//                Vector3 p = new Vector3(x, y, z) + Attractors[index].Center;

//                // finding the closest node to the generated Vector3
//                Tree.Node minNode = agent.Tree.Min(p);

//                // creating offset vector to new node
//                Vector3 v = new Vector3(minNode.p, p);
//                Vector3 pNew = minNode.p + v.Normalized * d;

//                // checking for collisions of the new node
//                bool collision = false;
//                if (CollisionCheck)
//                {
//                    foreach (var obst in Obstacles)
//                    {
//                        if (obst.Contains(pNew))
//                        {
//                            collision = true;
//                            break;
//                        }
//                    }
//                }

//                if (!collision)
//                {
//                    // solving IKP for new node
//                    Contestant.q = minNode.q;
//                    var res = Solver.Execute(Obstacles, Contestant, pNew, Contestant.Joints.Length);
//                    if (res.Item1 && !(CollisionCheck && res.Item4.Contains(true)))
//                    {
//                        // adding node to the tree
//                        Tree.Node node = new Tree.Node(minNode, Contestant.GripperPos, Contestant.q);
//                        agent.Tree.AddNode(node);
//                        if (pNew.DistanceTo(Attractors[index].Center) < Attractors[index].Radius)
//                        {
//                            // removing attractor if it has been hit
//                            if (index != 0)
//                                Attractors.RemoveAt(index);
//                            else
//                                Attractors[index].InliersCount++;
//                        }
//                    }
//                }

//                // stopping in case the main attractor has been hit
//                if (Attractors[0].InliersCount != 0)
//                    break;
//            }

//            // retrieving resultant path along with respective configurations
//            Tree.Node start = agent.Tree.Min(agent.Goal), node_curr = start;
//            List<Vector3> path = new List<Vector3>();
//            List<Vector> configs = new List<Vector>();
//            for (int i = start.Layer; i >= 0; i--)
//            {
//                if (node_curr.Layer == i)
//                {
//                    path.Add(node_curr.p);
//                    configs.Add(node_curr.q);
//                    if (node_curr.Parent != null)
//                    {
//                        int Vector3sNum = node_curr.Layer - node_curr.Parent.Layer - 1;
//                        if (Vector3sNum > 0)
//                        {
//                            Tree.Node[] nodes = Tree.Discretize(node_curr, node_curr.Parent, Vector3sNum);
//                            foreach (var node in nodes)
//                            {
//                                configs.Add(node.q);
//                            }
//                        }
//                    }

//                    node_curr = node_curr.Parent;
//                }
//            }

//            // reverting path so that it goes from root to goal
//            path.Reverse();
//            configs.Reverse();

//            return (path, configs);
//        }
//    }
//}
