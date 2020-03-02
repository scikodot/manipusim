using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using OpenTK;

namespace Logic
{
    // TODO: maybe convert to partial class?
    class Algorithm  // TODO: should be marked as abstract or virtual
    {
        public static Random Rng;
        public Obstacle[] Obstacles;
        public double Precision, StepSize;
        public int ParamNum, MaxTime, Time = 0;

        public Algorithm(Obstacle[] obstacles, int paramNum, double precision, double stepSize, int maxTime)
        {
            Rng = new Random();
            Obstacles = obstacles;
            ParamNum = paramNum;
            Precision = precision;
            StepSize = stepSize;
            MaxTime = maxTime;
        }

        public bool[] DetectCollisions(Manipulator manip)
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
            foreach (var obst in Obstacles)
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
    }

    class HillClimbing : Algorithm
    {
        public HillClimbing(Obstacle[] obstacles, int paramNum, double precision, double stepSize, int maxTime) : base(obstacles, paramNum, precision, stepSize, maxTime) { }

        public Tuple<bool, double, double[], bool[]> Execute(Manipulator agent, Point goal)
        {
            // initial parameters
            double[] qBest = Misc.CopyArray(agent.q);
            double dist = agent.DistanceTo(goal), init_dist = dist, k = 1;
            double minDist = double.PositiveInfinity;
            bool Converged = false;
            
            double[] dq = new double[ParamNum];
            double range = 0, stepNeg = 0, stepPos = 0;
            Dispatcher.Timer.Start();
            while (Time++ < MaxTime)
            {
                for (int i = 0; i < ParamNum; i++)
                {
                    // checking GC constraints
                    range = agent.Joints[i].qRanges[0] - agent.q[i] * 180 / Math.PI;
                    stepNeg = range <= -StepSize ? -StepSize : range;

                    range = agent.Joints[i].qRanges[1] - agent.q[i] * 180 / Math.PI;
                    stepPos = range >= StepSize ? StepSize : range;

                    // generating random GCs' offset
                    dq[i] = (stepNeg + Rng.NextDouble() * (stepPos - stepNeg)) * Math.PI / 180;
                    dq[i] *= k;
                }

                // retrieving score of the new configuration
                agent.q = qBest.Zip(dq, (t, s) => { return t + s; }).ToArray();
                double distNew = agent.DistanceTo(goal);

                if (distNew < dist)
                {
                    // updating agent's configuration if it's better than the previos one
                    qBest = Misc.CopyArray(agent.q);
                    minDist = dist = distNew;
                    k = dist / init_dist;
                }

                if (dist < Precision)
                {
                    // the algorithm has converged
                    Converged = true;
                    break;
                }
            }
            Dispatcher.Timer.Stop();
            Console.SetCursorPosition(0, 15);
            Console.WriteLine("IKP time: {0}; Real time: {1}", Time, Dispatcher.Timer.ElapsedTicks / 10);
            Dispatcher.Timer.Reset();

            // checking for collisions of the found configuration if the algorithm has converged
            bool[] Collisions = new bool[agent.q.Length - 1];
            if (Converged)
                Collisions = DetectCollisions(agent);

            // resetting timer
            Time = 0;

            return new Tuple<bool, double, double[], bool[]>(Converged, minDist, qBest, Collisions);
        }
    }

    class Jacobian : Algorithm
    {
        public Jacobian(Obstacle[] obstacles, int paramNum, double precision, double stepSize, int maxTime) : base(obstacles, paramNum, precision, stepSize, maxTime) { }

        public Tuple<bool, double, double[], bool[]> Execute(Manipulator agent, Point goal)
        {
            VectorN offset = new VectorN(agent.Joints.Length);
            for (int j = 0; j < 10; j++)
            {
                var grip = agent.GripperPos;
                Vector3 gPos = new Vector3((float)grip.x, (float)grip.y, (float)grip.z);
                Vector3 tPos = new Vector3((float)goal.x, (float)goal.y, (float)goal.z);
                Vector3 error = tPos - gPos;

                // get all joints parameters (positions/axes)
                var joints = agent.Joints;
                var dh = agent.DH;
                var jointsCount = agent.Joints.Length;

                Matrix4 model = Matrix4.Identity;

                Vector3[] jAxes = new Vector3[jointsCount];
                Vector3[] jPos = new Vector3[jointsCount];

                jAxes[0] = Vector3.UnitY;
                jPos[0] = Vector3.Zero;

                Vector4 initAxis = Vector4.UnitY;
                for (int i = 0; i < jointsCount; i++)  // TODO: all the manipulator structure logic should be incapsulated (elsewhere)
                {
                    model *= Matrix.RotateY((float)dh[i].theta(joints[i]));
                    model *= Matrix.Translate((float)dh[i].d * Vector3.UnitY);
                    model *= Matrix.RotateX((float)dh[i].alpha);
                    model *= Matrix.Translate((float)dh[i].r * Vector3.UnitX);

                    if (i < jointsCount - 1)
                    {
                        jAxes[i + 1] = new Vector3(model * initAxis).Normalized();
                        jPos[i + 1] = new Vector3(model.M14, model.M24, model.M34);
                    }
                }

                // calculate the Jacobian
                double[,] data = new double[6, jointsCount];
                for (int i = 0; i < jointsCount; i++)
                {
                    var elem = Vector3.Cross(jAxes[i], gPos - jPos[i]);
                    data[0, i] = elem.X;
                    data[1, i] = elem.Y;
                    data[2, i] = elem.Z;
                    data[3, i] = jAxes[i].X;
                    data[4, i] = jAxes[i].Y;
                    data[5, i] = jAxes[i].Z;
                }

                Matrix J = new Matrix(data);

                // get transpose of the Jacobian
                Matrix Jtr = Matrix.Transpose(J);

                // calculate GC offsets
                VectorN dq = Jtr * new VectorN(new double[6] { error.X, error.Y, error.Z, 0, 0, 0 });
                dq *= -0.1;

                // checking for collisions of the found configuration if the algorithm has converged
                agent.q = agent.q.Zip(dq.Data, (t, s) => t + s).ToArray();
                bool[] collisions = new bool[agent.q.Length - 1];
                collisions = DetectCollisions(agent);

                offset += dq;
                var dist = agent.GripperPos.DistanceTo(goal);

                if (j == 9)
                    return new Tuple<bool, double, double[], bool[]>(true, dist, offset.Data, collisions);
            }

            return null;
        }
    }
}