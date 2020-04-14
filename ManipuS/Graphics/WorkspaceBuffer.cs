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
        //new LinkData { Length = 1 },
        //new LinkData { Length = 1 },
        //new LinkData { Length = 1 },
    };

    public static System.Numerics.Vector3[][] JointAxes =
    {
        new System.Numerics.Vector3[]
        {
            System.Numerics.Vector3.UnitY,
            System.Numerics.Vector3.UnitZ,
            System.Numerics.Vector3.UnitZ,
            System.Numerics.Vector3.UnitY,
        }
        //new System.Numerics.Vector3[]
        //{
        //    System.Numerics.Vector3.UnitY,
        //    System.Numerics.Vector3.UnitX,
        //    System.Numerics.Vector3.UnitZ,
        //    System.Numerics.Vector3.UnitX,
        //    System.Numerics.Vector3.UnitZ,
        //    System.Numerics.Vector3.UnitX,
        //    System.Numerics.Vector3.UnitY,
        //},
    };

    public static System.Numerics.Vector3[][] JointPositions =
    {
        new System.Numerics.Vector3[]
        {
            new System.Numerics.Vector3(1, 0, 1),
            new System.Numerics.Vector3(1, 1.4f, 1),
            new System.Numerics.Vector3(1, 2.8f, 1),
            new System.Numerics.Vector3(1, 4.1f, 1),
        },
        new System.Numerics.Vector3[]
        {
            new System.Numerics.Vector3(-1, 0, -1),
            new System.Numerics.Vector3(-1, 1.4f, -1),
            new System.Numerics.Vector3(-1, 2.8f, -1),
            new System.Numerics.Vector3(-1, 4.1f, -1),
        }
        //new System.Numerics.Vector3[]
        //{
        //    new System.Numerics.Vector3(2, 0, -2),
        //    new System.Numerics.Vector3(2, 1.4f, -2),
        //    new System.Numerics.Vector3(2, 2.8f, -2),
        //    new System.Numerics.Vector3(2, 4.2f, -2),
        //    new System.Numerics.Vector3(2, 5.6f, -2),
        //    new System.Numerics.Vector3(2, 7.0f, -2),
        //    new System.Numerics.Vector3(2, 8.3f, -2),
        //},
        //new System.Numerics.Vector3[]
        //{
        //    new System.Numerics.Vector3(-2, 0, 2),
        //    new System.Numerics.Vector3(-2, 1.4f, 2),
        //    new System.Numerics.Vector3(-2, 2.8f, 2),
        //    new System.Numerics.Vector3(-2, 4.2f, 2),
        //    new System.Numerics.Vector3(-2, 5.6f, 2),
        //    new System.Numerics.Vector3(-2, 7.0f, 2),
        //    new System.Numerics.Vector3(-2, 8.3f, 2),
        //},
    };

    public static JointData[] JointBuffer =
    {
        new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180)
        },
        new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180)
        },
        new JointData
        {
            Length = 0.4f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180)
        },
        //new JointData
        //{
        //    Length = 0.4f,
        //    q = 0,
        //    qRanges = new System.Numerics.Vector2(-180, 180),
        //},
        //new JointData
        //{
        //    Length = 0.4f,
        //    q = 0,
        //    qRanges = new System.Numerics.Vector2(-180, 180),
        //},
        //new JointData
        //{
        //    Length = 0.4f,
        //    q = 0,
        //    qRanges = new System.Numerics.Vector2(-180, 180),
        //},
        new JointData
        {
            Length = 0.2f,
            q = 0,
            qRanges = new System.Numerics.Vector2(-180, 180)
        }
    };

    public static ManipData[] ManipBuffer =  // TODO: synchronize thread access while loading models
    {
        new ManipData
        {
            N = 3,
            Links = LinkBuffer.ToArray(),
            Joints = JointBuffer.ToArray(),
            JointAxes = JointAxes[0].ToArray(),
            JointPositions = JointPositions[0].ToArray(),
            Goal = new System.Numerics.Vector3(0, 0.5f, -1f),
            ShowTree = true
        },
        new ManipData
        {
            N = 3,
            Links = LinkBuffer.ToArray(),
            Joints = JointBuffer.ToArray(),
            JointAxes = JointAxes[0].ToArray(),
            JointPositions = JointPositions[1].ToArray(),
            Goal = new System.Numerics.Vector3(0, 0.5f, 1f),
            ShowTree = true
        }
        //new ManipData
        //{
        //    N = 6,
        //    Links = LinkBuffer.ToArray(),
        //    Joints = JointBuffer.ToArray(),
        //    JointAxes = JointAxes[0].ToArray(),
        //    JointPositions = JointPositions[0].ToArray(),
        //    Goal = new System.Numerics.Vector3(-1f, 3f, 1f),
        //    ShowTree = true
        //},
        //new ManipData
        //{
        //    N = 6,
        //    Links = LinkBuffer.ToArray(),
        //    Joints = JointBuffer.ToArray(),
        //    JointAxes = JointAxes[0].ToArray(),
        //    JointPositions = JointPositions[1].ToArray(),
        //    Goal = new System.Numerics.Vector3(-1f, 5f, -2f),
        //    ShowTree = true
        //},
    };

    public static ObstData[] ObstBuffer = 
    {
        new ObstData
        {
            Radius = 0.5f,
            Center = new System.Numerics.Vector3(0, 2, 0),
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
        //new ObstData
        //{
        //    Radius = 1f,
        //    Center = new System.Numerics.Vector3(0, 4, -1),
        //    PointsNum = 2000,
        //    ShowCollider = true
        //},
        //new ObstData
        //{
        //    Radius = 0.75f,
        //    Center = new System.Numerics.Vector3(0, 5, 0),
        //    PointsNum = 3000,
        //    ShowCollider = true
        //},
        //new ObstData
        //{
        //    Radius = 0.5f,
        //    Center = new System.Numerics.Vector3(-1, 8, 0),
        //    PointsNum = 1500,
        //    ShowCollider = true
        //},
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
