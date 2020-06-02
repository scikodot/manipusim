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
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public enum ControllerState
    {
        Aborted = -1,  // TODO: can be used for exceptions processing
        Idle = 0,
        Running = 1,
        Finished = 2
    }

    public class MotionController
    {
        public Manipulator Manipulator { get; }
        public PathPlanner PathPlanner { get; set; }
        public InverseKinematicsSolver InverseKinematicsSolver { get; set; }

        private float _deformThreshold = 0.1f;

        public ControllerState State { get; private set; } = ControllerState.Idle;
        public Stopwatch Timer { get; } = new Stopwatch();
        public Thread Thread { get; private set; }

        public MotionController(Manipulator agent, PathPlanner pathPlanner, InverseKinematicsSolver planSolver)
        {
            Manipulator = agent;
            PathPlanner = pathPlanner;
            InverseKinematicsSolver = planSolver;
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
                    ExecuteMotion(Manipulator.Goal);

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
            var ppRes = PathPlanner.Run(Manipulator, goal, InverseKinematicsSolver);

            // stop measuring planning time
            Timer.Stop();

            Manipulator.Path = ppRes.Path;

            // start motion control if a path has been found
            if (Manipulator.Path != null)
            {
                using (var manipulatorCopy = Manipulator.DeepCopy())
                {
                    // execute motion control
                    Path.Node gripperPos = Manipulator.Path.Current;
                    while (gripperPos.Child != null)
                    {
                        var current = gripperPos.Child;
                        while (current.Child != null)  // last point may not be deformed, since it is a goal point
                        {
                            for (int j = current.Points.Length - 1; j > 0; j--)
                            {
                                if (ObstacleHandler.ContainmentTest(current.Points[j], out Obstacle obstacle))
                                {
                                    //Deform(manipulatorCopy, obstacle, current, j);
                                }
                            }

                            current = current.Child;
                        }

                        //Discretize(manipulatorCopy, gripperPos);

                        gripperPos = Manipulator.Path.Current;
                    }
                }
            }
            else
            {
                // TODO: notify that the paths was not found?
            }
        }

        //private void Deform(Manipulator manipulator, Obstacle obstacle, Path.Node current, int joint)
        //{
        //    Vector3 dx = obstacle.Extrude(current.Points[joint]);

        //    Vector3 pNew = current.Points[joint] + dx;
        //    manipulator.q = current.q;
        //    VectorFloat cNew = manipulator.q + InverseKinematicsSolver.Execute(manipulator, pNew, joint).Item3;
        //    manipulator.q = cNew;
        //    Vector3[] dkpNew = manipulator.DKP;

        //    Manipulator.Path.ChangeNode(current, dkpNew, cNew);
        //}

        //private void Discretize(Manipulator manipulator, Path.Node gripperPos)
        //{
        //    Path.Node prev = gripperPos;
        //    Path.Node curr = gripperPos.Child;
        //    while (curr != null)
        //    {
        //        Vector3 prevPos = prev.Points[prev.Points.Length - 1];
        //        Vector3 currPos = curr.Points[curr.Points.Length - 1];
        //        if (currPos.DistanceTo(prevPos) > _deformThreshold)
        //        {
        //            Vector3 pPrev = (currPos + prevPos) / 2;
        //            manipulator.q = curr.q;
        //            VectorFloat cPrev = manipulator.q + InverseKinematicsSolver.Execute(manipulator, pPrev, Manipulator.Joints.Length - 1).Item3;
        //            manipulator.q = cPrev;
        //            Vector3[] dkpPrev = manipulator.DKP;

        //            Manipulator.Path.AddNode(new Path.Node(prev, dkpPrev, cPrev));
        //        }

        //        curr = curr.Child;
        //        prev = prev.Child;
        //    }
        //}
    }
}
