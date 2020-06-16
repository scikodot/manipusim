using System.Threading;
using System.Threading.Tasks;

using Logic.PathPlanning;
using Logic.InverseKinematics;

namespace Logic
{
    public enum ControllerState
    {
        Aborted = -1,
        Idle = 0,
        Running = 1,
    }

    public class Controller
    {
        public Manipulator Manipulator { get; }
        public InverseKinematicsSolver InverseKinematicsSolver { get; set; }
        public PathPlanner PathPlanner { get; set; }
        public MotionController MotionController { get; set; }

        private CancellationTokenSource _cancellationTokenSource;
        public Task PlanningTask { get; private set; }
        public Task ControlTask { get; private set; }

        public Controller(Manipulator manipulator, PathPlanner pathPlanner, InverseKinematicsSolver inverseKinematicsSolver, MotionController motionController)
        {
            Manipulator = manipulator;
            InverseKinematicsSolver = inverseKinematicsSolver;
            PathPlanner = pathPlanner;
            MotionController = motionController;
        }

        public void Run()
        {
            _cancellationTokenSource = new CancellationTokenSource();
            var cancellationToken = _cancellationTokenSource.Token;

            PlanningTask = Task.Run(() =>
            {
                var res = PathPlanner.Run(Manipulator, Manipulator.Goal, InverseKinematicsSolver, cancellationToken);
                Manipulator.Path = res.Path;
            }, cancellationToken);

            ControlTask = PlanningTask.ContinueWith(task =>
            {
                MotionController.Run(Manipulator, cancellationToken);
            }, cancellationToken);
        }

        public void Abort()
        {
            _cancellationTokenSource.Cancel();
        }
    }
}