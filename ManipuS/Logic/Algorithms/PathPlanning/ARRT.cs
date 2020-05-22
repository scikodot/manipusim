using Logic.InverseKinematics;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace Logic.PathPlanning
{
    class ARRT : RRT
    {
        private List<Attractor> _attractors;

        private int _attractorsCount = 5000;
        public ref int AttractorsCount => ref _attractorsCount;

        private int _trimPeriod;
        public ref int TrimPeriod => ref _trimPeriod;

        public ARRT(Manipulator manipulator, int maxTime, bool collisionCheck, float step, int attractorsCount, int trimPeriod) : base(maxTime, collisionCheck, step)
        {
            _attractorsCount = attractorsCount;
            _trimPeriod = trimPeriod;

            // create attractors
            _attractors = Attractor.Create(manipulator, _attractorsCount, _threshold);  // TODO: if attractors count can be changed from the outside, 
                                                                                        // then the generation should happen on execution or like an event
        }

        public override (int, Path) Execute(Manipulator agent, Vector3 goal, InverseKinematicsSolver solver)
        {
            // recalculate attractors' weights
            Attractor.RecalculateWeights(_attractors, agent, _threshold);

            Manipulator agentCopy = agent.DeepCopy();

            // create new tree
            agent.Tree = new Tree(new Tree.Node(null, agent.GripperPos, agent.q));  // TODO: consider creating local tree and returning it (for benchmarking or similar)

            // sort attractors
            _attractors = _attractors.OrderBy(a => a.Weight).ToList();

            // create necessary locals
            Attractor attractorFirst = _attractors.First();
            Attractor attractorLast = _attractors.Last();  // TODO: last attractor has too big radius (~5 units for 0.04 step); fix!

            float mu = attractorFirst.Weight;
            float sigma = (attractorLast.Weight - attractorFirst.Weight) / 3.0f;

            int iters = 0;
            while (iters++ < _maxTime)  // TODO: rename to TimeLimit?
            {
                //if (i % _trimPeriod == 0 && i != 0)
                //    agent.Tree.Trim(obstacles, agentCopy, solver);

                // generate normally distributed weight
                float num = RandomThreadStatic.NextGaussian(mu, sigma);

                // get the index of the most relevant attractor
                int index = _attractors.IndexOfNearest(num, a => a.Weight);

                // get point of the obtained attractor
                Vector3 sample = _attractors[index].Center;

                // find the closest node to that attractor
                Tree.Node nodeClosest = agent.Tree.Closest(sample);

                // get new tree node point
                Vector3 point = nodeClosest.Point + Vector3.Normalize(sample - nodeClosest.Point) * _step;

                if (!(_discardOutliers && ObstacleHandler.ContainmentTest(point)))
                {
                    // solve inverse kinematics for the new node
                    agentCopy.q = nodeClosest.q;
                    (var converged, _, var distance, var offset) = solver.Execute(agentCopy, point, agentCopy.Joints.Length - 1);
                    if (converged && !(_collisionCheck && agent.CollisionTest().Contains(true)))
                    {
                        // add node to the tree
                        Tree.Node node = new Tree.Node(nodeClosest, agentCopy.GripperPos, agentCopy.q);
                        agent.Tree.AddNode(node);
                    }
                }

                if (point.DistanceTo(_attractors[index].Center) < _attractors[index].Radius)
                {
                    if (index == 0)
                        // stop in case the main attractor has been hit
                        break;
                    else
                        // remove attractor if it has been hit
                        _attractors.RemoveAt(index);
                }
            }

            // retrieve resultant path along with respective configurations
            return (iters - 1, agent.Tree.GetPath(agentCopy, agent.Tree.Closest(goal)));  // TODO: refactor!
        }
    }
}