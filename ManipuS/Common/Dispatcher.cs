using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Threading;
using System.Collections.Concurrent;
using System.Numerics;
using System.Diagnostics;

using Logic;

// global class for communication between main thread (Window) and auxiliary threads (Model, Manager, etc.)
public static class Dispatcher  // TODO: threads are not synchronized; 
                                // when FPS drops, the algorithms fall apart because they cannot react to workspace changes in time,
                                // and that causes poor performance of the whole program
                                // FIX!!!
{
    public static List<Task> ActiveTasks = new List<Task>();

    public static ManualResetEvent ActionsDone = new ManualResetEvent(false);

    public static Stopwatch Timer = new Stopwatch();

    // actions that main thread has to execute
    public static ConcurrentQueue<Action> RenderActions = new ConcurrentQueue<Action>();

    // manipulators' calculating threads
    public static List<(Manipulator, Thread)> Threads { get; } = new List<(Manipulator, Thread)>();
    //public static List<bool> ThreadsRunning { get; } = new List<bool>();
    //public static List<Stopwatch> timers { get; } = new List<Stopwatch>();

    public static CancellationTokenSource tokenSource = new CancellationTokenSource();

    // obstacles' threads
    public static Timer obstThread;
    public static float time = 0;
    public static bool forward;

    public static void MoveObstacles(object state)
    {
        //float delta = 0.01f;
        //float dt;
        //if (forward)
        //{
        //    dt = delta;
        //    if (time > 1)
        //        forward = false;
        //}
        //else
        //{
        //    dt = -delta;
        //    if (time < -1)
        //        forward = true;
        //}
        //time += dt;

        //if (!Manager.Manipulators.All(x => x.Controller.State == ControllerState.Finished))
        //{
        //    Manager.Obstacles[0].Move(dt * Vector3.UnitX);
        //    //Manager.Obstacles[1].Move(dt * new Vector3(-1, 0, -1));
        //    //Manager.Obstacles[2].Move(-dt * new Vector3(-1, -1, -1));
        //}
    }

    public static void RunObstacles()
    {
        obstThread = new Timer(MoveObstacles, null, 0, 10);
    }

    //public static void AddThread(Manipulator manipulator)
    //{
    //    Threads.Add((manipulator, new Thread(() =>
    //    {
    //        try
    //        {
    //            // start measuring execution time
    //            manipulator.Controller.Timer.Start();

    //            // turn the controller on
    //            manipulator.Controller.State = ControllerState.Running;

    //            // execute manipulator control process
    //            manipulator.Controller.ExecuteMotion(manipulator.Goal);

    //            // turn the controller off
    //            manipulator.Controller.State = ControllerState.Finished;
    //        }
    //        catch (ThreadAbortException e)  // checking for abort query
    //        {
    //            // indicate that the process has been aborted
    //            manipulator.Controller.State = ControllerState.Aborted;
    //        }
    //        finally
    //        {
    //            // stop measuring execution time
    //            manipulator.Controller.Timer.Reset();
    //        }
    //    })
    //    {
    //        Name = $"Manipulator",
    //        IsBackground = true
    //    }));
    //}

    //public static void RunThreads()
    //{
    //    // running all threads
    //    foreach (var thread in Threads)
    //    {
    //        thread.Item2.Start();
    //    }
    //}

    //public static void AbortThreads()
    //{
    //    // aborting all running threads if presented
    //    if (Threads.Count != 0)
    //    {
    //        foreach (var thread in Threads)
    //        {
    //            if (thread.Item1.Controller.State == ControllerState.Running)
    //                thread.Item2.Abort();
    //        }

    //        // wait until all threads abort
    //        while (Threads.All(x => x.Item1.Controller.State == ControllerState.Running)) { }
    //    }
    //}
}
