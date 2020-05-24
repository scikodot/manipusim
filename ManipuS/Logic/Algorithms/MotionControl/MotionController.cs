using System.Collections.Generic;
using System.Numerics;
using Logic.PathPlanning;
using Logic.InverseKinematics;
using System.Diagnostics;
using System;
using System.Threading;
using System.Linq;
using Assimp;

namespace Logic
{
    public enum ControllerState
    {
        Aborted = -1,  // TODO: can be used for exceptions processing
        Idle = 0,
        Running = 1,
        Finished = 2
    }

    public class MotionController
    {
        public Manipulator Agent { get; }
        public Obstacle[] Obstacles { get; }

        public PathPlanner PathPlanner { get; set; }
        
        public InverseKinematicsSolver PlanSolver { get; set; }


        private InverseKinematicsSolver _controlSolver;
        private float _deformThreshold;

        public ControllerState State { get; private set; } = ControllerState.Idle;
        public Stopwatch Timer { get; } = new Stopwatch();
        public Thread Thread { get; private set; }

        public MotionController(Obstacle[] obstacles, Manipulator agent, PathPlanner pathPlanner, InverseKinematicsSolver planSolver, InverseKinematicsSolver controlSolver, float deformThreshold)
        {
            Obstacles = obstacles;
            Agent = agent;
            PathPlanner = pathPlanner;
            PlanSolver = planSolver;
            _controlSolver = controlSolver;
            _deformThreshold = deformThreshold;
        }

        public void Run()
        {
            // update thread
            UpdateThread();

            // run thread
            Thread.Start();
        }

        public void Abort()
        {
            if (State == ControllerState.Running)
                Thread.Abort();
        }

        private void UpdateThread()
        {
            Thread = new Thread(() =>  // TODO: consider changing to Tasks
            {
                try
                {
                    // start measuring execution time
                    Timer.Reset();
                    Timer.Start();

                    // turn the controller on
                    State = ControllerState.Running;

                    // execute manipulator control process
                    ExecuteMotion(Agent.Goal);

                    // turn the controller off
                    State = ControllerState.Finished;
                }
                catch (ThreadAbortException e)  // checking for abort query
                {
                    // indicate that the process has been aborted
                    State = ControllerState.Aborted;
                }
                finally
                {
                    // stop measuring execution time
                    Timer.Stop();
                }
            });
        }

        private void ExecuteMotion(Vector3 goal)
        {
            // execute path planning
            (_, var path) = PathPlanner.Run(Agent, goal, PlanSolver);
            Agent.Path = path;

            // start motion control if a path has been found
            if (Agent.Path != null)
            {
                using (var manipulatorCopy = Agent.DeepCopy())
                {
                    // execute motion control
                    Path.Node gripperPos = Agent.Path.Current;
                    while (gripperPos.Child != null)
                    {
                        var current = gripperPos.Child;
                        while (current.Child != null)  // last point may not be deformed, since it is a goal point
                        {
                            for (int j = current.Points.Length - 1; j > 0; j--)
                            {
                                foreach (var obst in Obstacles)
                                {
                                    if (obst.Contains(current.Points[j]))
                                    {
                                        //Deform(obst, contestant, current, j);
                                        break;
                                    }
                                }
                            }

                            current = current.Child;
                        }

                        //Discretize(contestant, gripperPos);

                        gripperPos = Agent.Path.Current;
                    }
                }
            }
        }

        private void Deform(Obstacle obstacle, Manipulator contestant, Path.Node current, int joint)
        {
            Vector3 dx = obstacle.Extrude(current.Points[joint]);

            Vector3 pNew = current.Points[joint] + dx;
            contestant.q = current.q;
            MathNet.Numerics.LinearAlgebra.Vector<float> cNew = contestant.q + _controlSolver.Execute(contestant, pNew, joint).Item3;
            contestant.q = cNew;
            Vector3[] dkpNew = contestant.DKP;

            Agent.Path.ChangeNode(current, dkpNew, cNew);
        }

        private void Discretize(Manipulator contestant, Path.Node gripperPos)
        {
            Path.Node prev = gripperPos;
            Path.Node curr = gripperPos.Child;
            while (curr != null)
            {
                Vector3 prevPos = prev.Points[prev.Points.Length - 1];
                Vector3 currPos = curr.Points[curr.Points.Length - 1];
                if (currPos.DistanceTo(prevPos) > _deformThreshold)
                {
                    Vector3 pPrev = (currPos + prevPos) / 2;
                    contestant.q = curr.q;
                    MathNet.Numerics.LinearAlgebra.Vector<float> cPrev = contestant.q + _controlSolver.Execute(contestant, pPrev, Agent.Joints.Length - 1).Item3;
                    contestant.q = cPrev;
                    Vector3[] dkpPrev = contestant.DKP;

                    Agent.Path.AddNode(new Path.Node(prev, dkpPrev, cPrev));
                }

                curr = curr.Child;
                prev = prev.Child;
            }
        }
    }
}
