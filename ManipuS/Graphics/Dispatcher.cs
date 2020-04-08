using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using Logic;
using System.Threading;

// global class for communication between main thread (Window) and auxiliary threads (Model, Manager, etc.)
public static class Dispatcher
{
    public static List<Task> ActiveTasks = new List<Task>();

    public static ManualResetEvent ActionsDone = new ManualResetEvent(false);

    public static Random Rng = new Random();
    public static Stopwatch Timer = new Stopwatch();

    // actions that main thread has to execute
    public static Queue<Action> ActionsQueue = new Queue<Action>();

    // manipulators' calculating threads
    public static Thread[] threads;
    public static bool[] ThreadsRunning;
    public static Task[] tasks;
    public static CancellationTokenSource tokenSource = new CancellationTokenSource();
    public static Stopwatch[] timers;

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
