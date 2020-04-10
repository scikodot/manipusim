using Logic;
using System;
using System.Linq;

// buffer that accepts input data from the GUI, which is later used by the Manager
public class WorkspaceBuffer  // TODO: replace our Vectors with System.Numerics.Vectors?
{
    public static LinkData[] LinkBuffer =
    {
        new LinkData { Length = 1 },
        new LinkData { Length = 1 },
        new LinkData { Length = 1 },
        new LinkData { Length = 1 },
        new LinkData { Length = 1 },
        new LinkData { Length = 1 }
    };

    public static JointData[] JointBuffer =
    {
        new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180),
            Axis = System.Numerics.Vector3.UnitY,
            Position = new System.Numerics.Vector3(0, 0, 0)
        },
        new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180),
            Axis = System.Numerics.Vector3.UnitZ,
            Position = new System.Numerics.Vector3(0, 1.4f, 0)
        },
        new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180),
            Axis = System.Numerics.Vector3.UnitZ,
            Position = new System.Numerics.Vector3(0, 2.8f, 0)
        },
        new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180),
            Axis = System.Numerics.Vector3.UnitZ,
            Position = new System.Numerics.Vector3(0, 4.2f, 0)
        },
        new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180),
            Axis = System.Numerics.Vector3.UnitZ,
            Position = new System.Numerics.Vector3(0, 5.6f, 0)
        },
        new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180),
            Axis = System.Numerics.Vector3.UnitZ,
            Position = new System.Numerics.Vector3(0, 7.0f, 0)
        },
        new JointData
        {
            Length = 0.2f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180),
            Axis = System.Numerics.Vector3.UnitY,
            Position = new System.Numerics.Vector3(0, 8.3f, 0)
        }
    };

    public static ManipData[] ManipBuffer =  // TODO: synchronize thread access while loading models
    {
        new ManipData
        {
            N = 6,
            Base = new System.Numerics.Vector3(0, 0, 0),
            Links = LinkBuffer.ToArray(),
            Joints = JointBuffer.ToArray(),
            Goal = new System.Numerics.Vector3(0, 4f, 3f),
            ShowTree = true
        },
        //new ManipData
        //{
        //    N = 3,
        //    Base = new System.Numerics.Vector3(0, 0, 0),
        //    Links = LinkBuffer.ToArray(),
        //    Joints = JointBuffer.ToArray(),
        //    Goal = new System.Numerics.Vector3(0, 0.5f, -1f),
        //    ShowTree = true
        //}
    };

    public static ObstData[] ObstBuffer = 
    {
        new ObstData
        {
            Radius = 0.5f,
            Center = new System.Numerics.Vector3(0, 4, 2),
            PointsNum = 2000,
            ShowCollider = true
        },
        /*new ObstData
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
        InverseKinematicsSolverID = 0,
        Precision = 0.02f,
        StepSize = 2,
        MaxTime = 300,

        PathPlannerID = 0,
        AttrNum = 5000,
        k = 10000,
        d = 0.04f
    };

    public static void ConfigureArrays(int manipIndex)
    {
        int N = ManipBuffer[manipIndex].N;
        int Nprev = ManipBuffer[manipIndex].Links.Length;

        Array.Resize(ref ManipBuffer[manipIndex].Links, N);
        if (N > Nprev)
            ManipBuffer[manipIndex].Links.Fill(LinkBuffer[0], N);

        Array.Resize(ref ManipBuffer[manipIndex].Joints, N + 1);
        if (N > Nprev)
            ManipBuffer[manipIndex].Joints.Fill(JointBuffer[0], N);
        ManipBuffer[manipIndex].Joints[N] = JointBuffer[JointBuffer.Length - 1];
    }
}
