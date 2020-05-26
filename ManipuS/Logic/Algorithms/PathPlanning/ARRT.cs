using Logic.InverseKinematics;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace Logic.PathPlanning
{
    class ARRT : RRT
    {
        protected static int _attractorsCountDefault = 5000;
        protected static int _trimPeriodDefailt = 1000;

        protected List<Attractor> _attractors;

        protected int _attractorsCount;
        public ref int AttractorsCount => ref _attractorsCount;

        protected int _trimPeriod;
        public ref int TrimPeriod => ref _trimPeriod;

        public ARRT(Manipulator manipulator, int maxIterations, bool collisionCheck, float step, float threshold, 
            bool showTree, bool discardOutliers, int attractorsCount, int trimPeriod) : 
            base(maxIterations, collisionCheck, step, threshold, showTree, discardOutliers)
        {
            _attractorsCount = attractorsCount;
            _trimPeriod = trimPeriod;

            // create attractors
            _attractors = Attractor.Create(manipulator, _attractorsCount, _threshold);  // TODO: if attractors count can be changed from the outside, 
                                                                                        // then the generation should happen on execution or like an event
        }

        public static ARRT Default(Manipulator manipulator)
        {
            return new ARRT(manipulator, _maxIterationsDefault, _collisionCheckDefault, _stepDefault, _thresholdDefault, 
                _showTreeDefault, _discardOutliersDefault, _attractorsCountDefault, _trimPeriodDefailt);
        }

        protected override (int, Path) RunAbstract(Manipulator manipulator, Vector3 goal, InverseKinematicsSolver solver)
        {
            // recalculate attractors' weights
            Attractor.RecalculateWeights(_attractors, manipulator, _threshold);

            // create new tree
            Tree = new Tree(new Tree.Node(null, manipulator.GripperPos, manipulator.q));

            // sort attractors
            _attractors = _attractors.OrderBy(a => a.Weight).ToList();

            // create necessary locals
            Attractor attractorFirst = _attractors.First();
            Attractor attractorLast = _attractors.Last();  // TODO: last attractor has too big radius (~5 units for 0.04 step); fix!

            float mu = attractorFirst.Weight;
            float sigma = (attractorLast.Weight - attractorFirst.Weight) / 3.0f;

            int iters = 0;
            while (iters++ < _maxIterations)  // TODO: rename to TimeLimit?
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
                Tree.Node nodeClosest = Tree.Closest(sample);

                // get new tree node point
                Vector3 point = nodeClosest.Point + Vector3.Normalize(sample - nodeClosest.Point) * _step;

                if (!(_discardOutliers && ObstacleHandler.ContainmentTest(point, out _)))
                {
                    // solve inverse kinematics for the new node
                    manipulator.q = nodeClosest.q;
                    (var converged, _, var distance, var offset) = solver.Execute(manipulator, point, manipulator.Joints.Length - 1);
                    if (converged && !(_collisionCheck && manipulator.CollisionTest().Contains(true)))
                    {
                        // add node to the tree
                        Tree.Node node = new Tree.Node(nodeClosest, manipulator.GripperPos, manipulator.q);
                        Tree.AddNode(node);
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
            return (iters - 1, Tree.GetPath(manipulator, Tree.Closest(goal)));  // TODO: refactor!
        }
    }
}