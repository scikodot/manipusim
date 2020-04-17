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

    public static Random Rng = new Random();
    public static Stopwatch Timer = new Stopwatch();

    // actions that main thread has to execute
    public static ConcurrentQueue<Action> ActionsQueue = new ConcurrentQueue<Action>();

    // manipulators' calculating threads
    public static Thread[] threads;
    public static bool[] ThreadsRunning;
    public static Task[] tasks;
    public static CancellationTokenSource tokenSource = new CancellationTokenSource();
    public static Stopwatch[] timers;

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

    public static void UpdateThreads()  // TODO: for WaitHandles Tasks would be better than Threads
    {
        // enabling/disabling specific threads for calculating paths for manipulators
        threads = new Thread[Manager.Manipulators.Length];
        ThreadsRunning = new bool[Manager.Manipulators.Length];
        for (int i = 0; i < threads.Length; i++)
        {
            int index = i;
            threads[i] = new Thread(() =>
            {
                timers[index].Start();
                try
                {
                    ThreadsRunning[index] = true;
                    Manager.Manipulators[index].Controller.Execute(Manager.Manipulators[index].Goal);
                    ThreadsRunning[index] = false;
                }
                catch (ThreadAbortException e)  // checking for abort query
                {
                    ThreadsRunning[index] = false;
                }
                finally
                {
                    timers[index].Stop();
                }
            })
            {
                Name = $"Manipulator {index}",
                IsBackground = true
            };
        }
    }

    public static void RunThreads()
    {
        // running all threads
        for (int i = 0; i < threads.Length; i++)
        {
            threads[i].Start();
        }
    }

    public static void AbortThreads()
    {
        // aborting all running threads if presented
        if (threads != null)
        {
            for (int i = 0; i < threads.Length; i++)
            {
                if (ThreadsRunning[i] == true)
                    threads[i].Abort();
            }

            // wait until all threads abort
            while (ThreadsRunning.Contains(true)) { }
        }
    }
}
