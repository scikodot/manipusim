using System;
using System.Collections.Generic;
using System.Linq;

namespace Logic.InverseKinematics
{
    class IKSolver
    {
        protected Obstacle[] Obstacles;
        protected float Precision, StepSize;
        protected int MaxTime;

        protected IKSolver(Obstacle[] obstacles, float precision, float stepSize, int maxTime)
        {
            Obstacles = obstacles;
            Precision = precision;
            StepSize = stepSize;
            MaxTime = maxTime;
        }

        public bool[] DetectCollisions(Manipulator manip, Obstacle[] obstacles)
        {
            List<Vector3> joints = manip.DKP.ToList();

            // removing duplicates
            for (int i = 0; i < joints.Count - 1; i++)
            {
                if (joints[i].DistanceTo(joints[i + 1]) == 0)
                    joints.RemoveAt(i + 1);
            }

            // representing manipulator as a sequence of actuators
            List<Vector3> actuators = new List<Vector3>();
            int actuatorsNum = 50;
            for (int i = 0; i < joints.Count - 1; i++)
            {
                actuators.Add(joints[i]);
                for (int j = 0; j < actuatorsNum; j++)
                {
                    actuators.Add(new Vector3
                    (
                        joints[i].X + (j + 1) * (joints[i + 1].X - joints[i].X) / (actuatorsNum + 1),
                        joints[i].Y + (j + 1) * (joints[i + 1].Y - joints[i].Y) / (actuatorsNum + 1),
                        joints[i].Z + (j + 1) * (joints[i + 1].Z - joints[i].Z) / (actuatorsNum + 1)
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

        public virtual (bool, float, Vector, bool[]) Execute(Manipulator agent, Vector3 goal, int joint)
        {
            return default;
        }
    }
}
