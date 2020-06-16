using System;
using System.Collections.Generic;
using System.Numerics;

namespace Logic
{
    public class Attractor  // TODO: maybe struct would be better?
    {
        public Vector3 Center { get; private set; }
        public float Weight { get; private set; }
        public float Radius { get; private set; }

        public Attractor(Vector3 center)
        {
            Center = center;
        }

        public Attractor(Vector3 center, float weight, float radius)
        {
            Center = center;
            Weight = weight;
            Radius = radius;
        }

        public static List<Attractor> Create(Manipulator manipulator, int count, float goalThreshold, bool discardOutliers = true)
        {
            var attractors = new List<Attractor>();

            //// adding goal attractor
            //Vector3 attrPoint = manipulator.Goal;

            //attractors.Add(new Attractor(attrPoint));

            // adding ancillary attractors
            while (attractors.Count < count)
            {
                // generating attractor point
                Vector3 point = RandomThreadStatic.NextPointSphere(manipulator.WorkspaceRadius) + manipulator.Base;

                // checking whether the attractor is inside any obstacle or not if requested
                if (!(discardOutliers && ObstacleHandler.ContainmentTest(point, out _)))  // TODO: consider creating a list of bad attractors; they may serve as repulsion points
                {
                    // adding attractor to the list
                    Vector3 attrPoint = point;
                    attractors.Add(new Attractor(attrPoint));
                }
            }

            return attractors;
        }

        private static float CalculateAttractorWeight(Manipulator manipulator, Vector3 point)
        {
            return manipulator.DistanceTo(point) + manipulator.Goal.DistanceTo(point);
        }

        private static float CalculateAttractorRadius(Manipulator manipulator, float weight, float goalThreshold)
        {
            return goalThreshold * weight / manipulator.DistanceTo(manipulator.Goal);  /*(float)Math.Pow(weight / manipulator.DistanceTo(manipulator.Goal), 2);*/
        }

        public static void RecalculateWeights(List<Attractor> attractors, Manipulator manipulator, float goalThreshold)
        {
            foreach (var attractor in attractors)
            {
                attractor.Weight = CalculateAttractorWeight(manipulator, attractor.Center);
                attractor.Radius = CalculateAttractorRadius(manipulator, attractor.Weight, goalThreshold);
            }
        }
    }
}
