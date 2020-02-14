using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Logic;

// global class for communication between main thread (Window) and auxiliary threads (Model, Manager, etc.)
static class Dispatcher
{
    // actions that main thread has to execute
    public static Queue<Action> ActionsQueue = new Queue<Action>();

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
                }
        };

        public static ObstData[] ObstBuffer =
        {
                /*new ObstData
                {
                    r = 1,
                    c = new System.Numerics.Vector3(0, 2f, 0),
                    points_num = 2000,

                    ShowBounding = true
                },
                new ObstData
                {
                    r = 1,
                    c = new System.Numerics.Vector3(-2.2f, 3.5f, 0),
                    points_num = 2000,

                    ShowBounding = true
                },
                new ObstData
                {
                    r = 1,
                    c = new System.Numerics.Vector3(-2.2f, 0f, -1.5f),
                    points_num = 2000,

                    ShowBounding = true
                },
                new ObstData
                {
                    r = 2.5f,
                    c = new System.Numerics.Vector3(0, 1f, -6f),
                    points_num = 6000,

                    ShowBounding = true
                },
                new ObstData
                {
                    r = 0.75f,
                    c = new System.Numerics.Vector3(-1.6f, 2f, -2f),
                    points_num = 1500,

                    ShowBounding = true
                }*/
        };

        public static AlgData AlgBuffer = new AlgData
        {
            AttrNum = 10000,

            Precision = 0.02f,
            StepSize = 3,
            MaxTime = 300,

            k = 10000,
            d = 0.08f
        };
    }
}
