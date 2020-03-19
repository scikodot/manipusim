using System;
using System.Collections.Generic;
using System.Linq;

namespace Logic.InverseKinematics
{
    class IKSolver
    {
        protected Obstacle[] Obstacles;
        protected double Precision, StepSize;
        protected int MaxTime;

        protected IKSolver(Obstacle[] obstacles, double precision, double stepSize, int maxTime)
        {
            Obstacles = obstacles;
            Precision = precision;
            StepSize = stepSize;
            MaxTime = maxTime;
        }

        public bool[] DetectCollisions(Manipulator manip, Obstacle[] obstacles)
        {
            List<Point> joints = manip.DKP.ToList();

            // removing duplicates
            for (int i = 0; i < joints.Count - 1; i++)
            {
                if (joints[i].DistanceTo(joints[i + 1]) == 0)
                    joints.RemoveAt(i + 1);
            }

            // representing manipulator as a sequence of actuators
            List<Point> actuators = new List<Point>();
            int actuatorsNum = 50;
            for (int i = 0; i < joints.Count - 1; i++)
            {
                actuators.Add(joints[i]);
                for (int j = 0; j < actuatorsNum; j++)
                {
                    actuators.Add(new Point
                    (
                        joints[i].x + (j + 1) * (joints[i + 1].x - joints[i].x) / (actuatorsNum + 1),
                        joints[i].y + (j + 1) * (joints[i + 1].y - joints[i].y) / (actuatorsNum + 1),
                        joints[i].z + (j + 1) * (joints[i + 1].z - joints[i].z) / (actuatorsNum + 1)
                    ));
                }
            }
            actuators.Add(joints[joints.Count - 1]);

            // checking collisions for all actuators
            bool[] collisions = new bool[joints.Count - 1];
            foreach (var obst in obstacles)
            {
                for (int i = 0; i < actuators.Count; i++)
                {
                    if (obst.Contains(actuators[i]))
                    {
                        collisions[(i - 1) / (actuatorsNum + 1)] = true;
                        goto CollisionDetected;
                    }
                }
            }
        CollisionDetected:

            return collisions;
        }

        public virtual (bool, double, double[], bool[]) Execute(Manipulator agent, Point goal)
        {
            return default;
        }
    }
}
