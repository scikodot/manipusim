using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using Logic;
using System.Threading;

// global class for communication between main thread (Window) and auxiliary threads (Model, Manager, etc.)
static class Dispatcher
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
                    //Manager.Plan(Manager.Manipulators[index]);
                    Manager.Manipulators[index].controller.Execute(Manager.Manipulators[index].Goal);
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

    // buffer that accepts input data from the GUI, which is later used by the Manager
    public static class WorkspaceBuffer
    {
        public static LinkData[] LinkBuffer =
        {
                new LinkData
                {
                    Length = 1
                },
                new LinkData
                {
                    Length = 1
                },
                new LinkData
                {
                    Length = 1
                }
        };

        public static JointData[] JointBuffer =
        {
                new JointData
                {
                    Length = 0.4f,
                    q = 0,
                    q_ranges = new System.Numerics.Vector2(-180, 180),

                    ShowTree = true
                },
                new JointData
                {
                    Length = 0.4f,
                    q = 0,
                    q_ranges = new System.Numerics.Vector2(-180, 180),

                    ShowTree = true
                },
                new JointData
                {
                    Length = 0.4f,
                    q = 0,
                    q_ranges = new System.Numerics.Vector2(-180, 180),

                    ShowTree = true
                },
                new JointData
                {
                    Length = 0.25f,
                    q = 0,
                    q_ranges = new System.Numerics.Vector2(-180, 180),

                    ShowTree = true
                }
        };

        public static ObstData[] ObstBuffer =
        {
                new ObstData
                {
                    r = 0.5f,
                    c = new System.Numerics.Vector3(0, 2f, 1f),
                    Vector3s_num = 2000,

                    ShowBounding = true
                }
                /*new ObstData
                {
                    r = 1,
                    c = new System.Numerics.Vector3(-2.2f, 3.5f, 0),
                    Vector3s_num = 2000,

                    ShowBounding = true
                },
                new ObstData
                {
                    r = 1,
                    c = new System.Numerics.Vector3(-2.2f, 0f, -1.5f),
                    Vector3s_num = 2000,

                    ShowBounding = true
                },
                new ObstData
                {
                    r = 2.5f,
                    c = new System.Numerics.Vector3(0, 1f, -6f),
                    Vector3s_num = 6000,

                    ShowBounding = true
                },
                new ObstData
                {
                    r = 0.75f,
                    c = new System.Numerics.Vector3(-1.6f, 2f, -2f),
                    Vector3s_num = 1500,

                    ShowBounding = true
                }*/
        };

        public static AlgData AlgBuffer = new AlgData
        {
            AttrNum = 10000,

            Precision = 0.02f,
            StepSize = 2,
            MaxTime = 300,

            k = 10000,
            d = 0.04f
        };
    }
}
