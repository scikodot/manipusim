using System.Numerics;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;

using Logic.PathPlanning;
using Logic.InverseKinematics;
using System;

namespace Logic
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public class MotionController
    {
        private float _deformThreshold = 0.1f;

        public ControllerState State { get; private set; } = ControllerState.Idle;
        public Stopwatch Timer { get; } = new Stopwatch();
        public Task Task { get; private set; }

        public MotionController()
        {
            
        }

        public static MotionController Default()
        {
            return new MotionController();
        }

        public void Run(Manipulator manipulator, CancellationToken cancellationToken = default)
        {
            try
            {
                // start measuring execution time
                Timer.Restart();

                // turn the controller on
                State = ControllerState.Running;

                // start motion control if a path has been found
                if (manipulator.Path != null)
                {
                    using (var manipulatorCopy = manipulator.DeepCopy())
                    {
                        // execute motion control
                        Path.Node gripperPos = manipulator.Path.Current;
                        while (gripperPos.Child != null)
                        {
                            cancellationToken.ThrowIfCancellationRequested();

                            var current = gripperPos.Child;
                            while (current.Child != null)  // last point may not be deformed, since it is a goal point
                            {
                                for (int j = current.Points.Length - 1; j > 0; j--)
                                {
                                    /*if (ObstacleHandler.ContainmentTest(current.Points[j], out Obstacle obstacle))
                                    {
                                        //Deform(manipulatorCopy, obstacle, current, j);
                                    }*/
                                }

                                current = current.Child;
                            }

                            //Discretize(manipulatorCopy, gripperPos);

                            gripperPos = manipulator.Path.Current;
                        }
                    }
                }

                // turn the controller off
                State = ControllerState.Idle;
            }
            catch (OperationCanceledException oce)
            {
                // indicate that the process has been aborted
                State = ControllerState.Aborted;
            }
            finally
            {
                // stop measuring execution time
                Timer.Stop();
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
