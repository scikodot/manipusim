using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.GraphicsLibraryFramework;
using OpenTK.Windowing.Desktop;
using OpenTK.Graphics.OpenGL4;
using ImGuiNET;

using Logic;
using Logic.PathPlanning;
using Physics;

using Matrix4 = OpenTK.Mathematics.Matrix4;
using Vector3 = OpenTK.Mathematics.Vector3;
using Vector4 = OpenTK.Mathematics.Vector4;
using System.Reflection;

namespace Graphics
{
    public class MainWindow : GameWindow
    {
        // handlers of input, graphics, entities, etc.
        public InputHandler InputHandler { get; private set; }
        public ShaderHandler ShaderHandler { get; private set; }
        public PhysicsHandler PhysicsHandler { get; private set; }
        public ManipulatorHandler ManipulatorHandler { get; private set; }
        public ObstacleHandler ObstacleHandler { get; private set; }

        public Camera Camera { get; private set; }
        private readonly Vector3 _cameraDefaultPosition = new(-5, 3, 5);
        private readonly Vector2 _cameraDefaultOrientation = new(-15, -45);

        public Vector2i ViewportOrigin => new((int)(0.25f * Size.X), 0);
        public Vector2i ViewportSize => new() { X = (int)(0.75f * Size.X), Y = Size.Y };
        public float ViewportAspectRatio => (float)ViewportSize.X / ViewportSize.Y;

        public static readonly List<Model> _goalModels = new List<Model>();  // TODO: consider distributing to the appropriate classes and checking whether the types (Model, etc.) are available at runtime
        //private static Model _bezierPoints;

        private float time = 0;
        private bool forward;

        private Collider[] _dummyColliders;
        private Thread[] _dummyTasks;

        public bool enter;

        //private static GhostObject ghostObject;
        //private static Model ghostObjectModel;
        //private static GhostPairCallback ghostCallback;
        //private static Manipulator copy;
        //private static HingeConstraint hinge;

        //private static RigidBody doorBody;
        //private static Model doorModel;
        //private static HingeConstraint doorHinge;

        //Model Crytek;

        // ImGUI variables
        private MainWindowImGui _imGui;

        public Thread MainThread { get; } = Thread.CurrentThread;  // TODO: move to Dispatcher?

        public MainWindow(GameWindowSettings gameWindowSettings, NativeWindowSettings nativeWindowSettings) : 
            base(gameWindowSettings, nativeWindowSettings)
        {
            // InputHandler is initialized first because it holds all paths
            InputHandler = new InputHandler(this);
            ShaderHandler = new ShaderHandler(this);
            PhysicsHandler = new PhysicsHandler(this);
            ManipulatorHandler = new ManipulatorHandler(this);
            ObstacleHandler = new ObstacleHandler(this);
        }

        #region LOAD
        protected override void OnLoad()
        {
            //var unptr = Assimp.Unmanaged.AssimpLibrary.Instance.ImportFile(JointPath, Assimp.PostProcessSteps.None, Assimp.Unmanaged.AssimpLibrary.Instance.CreatePropertyStore());
            //var manptr = Assimp.Scene.FromUnmanagedScene(unptr);
            //var ptr = Assimp.Unmanaged.AssimpLibrary.Instance.ApplyPostProcessing(unptr, Assimp.PostProcessSteps.Triangulate);
            //manptr = Assimp.Scene.FromUnmanagedScene(ptr);

            _dummyColliders = new Collider[100];
            _dummyTasks = new Thread[100];
            
            //for (int i = 0; i < 2; i++)
            //{
            //    int index = i;
            //    _dummyColliders[index] = PhysicsHandler.CreateKinematicCollider(new BoxShape(0.5f));
            //    _dummyTasks[index] = new Thread(() =>
            //    {
            //        while (true)
            //        {
            //            var mat = Matrix.Identity;
            //            _dummyColliders[index].Body.WorldTransform = mat;
            //            _dummyColliders[index].Body.MotionState.SetWorldTransform(ref mat);
            //        }
            //    });
            //    _dummyTasks[index].Start();
            //}

            ManipulatorHandler.LoadDefaultModels();

            // attach ImGUI to this window
            _imGui = new MainWindowImGui(this);  // TODO: make static?

            // Camera is 6 units back and has the proper aspect ratio
            Camera = new Camera(_cameraDefaultPosition, _cameraDefaultOrientation, ViewportAspectRatio);

            // subscribe to the events
            InputHandler.SelectedObjectChanged += _imGui.OnSelectedObjectChanged;
            InputHandler.CameraPositionChanged += Camera.OnCameraMove;
            InputHandler.CameraOrientationChanged += Camera.OnCameraRotate;
            InputHandler.CameraZoomChanged += Camera.OnCameraZoom;

            //ObstacleHandler.Add(new Obstacle(new Model(new Mesh[]
            //    {
            //        Primitives.Cone(1, 2, 50, new MeshMaterial
            //        {
            //            Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
            //            Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
            //            Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
            //            Shininess = 8
            //        })
            //    }), PhysicsHandler.CreateDynamicCollider(new ConeShape(1, 2), 1, Matrix.Translation(-1.5f, 3, 0))));  // TODO: cone's rigid body center is not the circle center, but the center of mass; fix

            //Cubes = new Obstacle[3];
            //for (int i = 0; i < 3; i++)
            //{
            //    Matrix stateInit = default;
            //    switch (i)
            //    {
            //        case 0:
            //            stateInit = Matrix.Translation(0.0f, 3.0f, 0.0f);
            //            break;
            //        case 1:
            //            stateInit = Matrix.Translation(0.5f, 4.5f, 0.0f);
            //            break;
            //        case 2:
            //            stateInit = Matrix.Translation(1.0f, 6.0f, 0.0f);
            //            break;
            //    }

            //    Cubes[i] = new Obstacle(Primitives.Cube(0.5f, 0.5f, 0.5f, new MeshMaterial
            //    {
            //        Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
            //        Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
            //        Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
            //        Shininess = 8
            //    }), PhysicsHandler.CreateDynamicCollider(new BoxShape(0.5f, 0.5f, 0.5f), 1, stateInit));
            //}

            //for (int i = 0; i < 100; i++)
            //{
            //    ObstacleHandler.AddDefault(ObstacleShape.Sphere);
            //}
            //ObstacleHandler.AddDefault(ObstacleShape.Sphere);

            //ObstacleHandler.Add(new Obstacle(Primitives.Sphere(0.5f, 100, 100, new MeshMaterial
            //{
            //    Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
            //    Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
            //    Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
            //    Shininess = 8
            //}), PhysicsHandler.CreateKinematicCollider(new SphereShape(0.5f), Matrix.Translation(0, 3, -1.5f))));

            //ObstacleHandler.Add(new Obstacle(Primitives.Cube(0.5f, 0.5f, 0.5f, new MeshMaterial
            //{
            //    Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
            //    Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
            //    Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
            //    Shininess = 8
            //}), PhysicsHandler.CreateKinematicCollider(new BoxShape(0.5f, 0.5f, 0.5f))));

            //ObstacleHandler.Add(new Obstacle(new Model(new Mesh[]
            //{
            //    Primitives.Cylinder(0.15f, 1f, 1f, 50, new MeshMaterial
            //    {
            //        Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
            //        Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
            //        Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
            //        Shininess = 8
            //    })
            //}), PhysicsHandler.CreateKinematicCollider(new CylinderShape(0.15f, 1f, 0.15f))));

            //ObstacleHandler.Add(new Obstacle(Primitives.Cylinder(0.25f, 1, 1, 50, new MeshMaterial
            //{
            //    Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
            //    Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
            //    Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
            //    Shininess = 8
            //}), PhysicsHandler.CreateDynamicCollider(new CylinderShape(0.25f, 1, 0.25f), 1, Matrix.Translation(0, 10, 0))));

            //ghostCallback = new GhostPairCallback();
            //PhysicsHandler.World.Broadphase.OverlappingPairCache.SetInternalGhostPairCallback(ghostCallback);

            //ghostObject = new GhostObject();
            //ghostObject.CollisionShape = new BoxShape(new BulletSharp.Math.Vector3(1, 1, 1));
            //ghostObject.WorldTransform = Matrix.Translation(new BulletSharp.Math.Vector3(0, 4, 0));
            //ghostObject.CollisionFlags |= CollisionFlags.NoContactResponse;
            //PhysicsHandler.World.AddCollisionObject(ghostObject, CollisionFilterGroups.SensorTrigger, CollisionFilterGroups.AllFilter & ~CollisionFilterGroups.SensorTrigger);

            //ghostObjectModel = Primitives.Cube(1, 1, 1, new MeshMaterial
            //{
            //    Diffuse = new Vector4(0, 1, 0, 0.3f)
            //});

            //var doorShape = new BoxShape(2.0f, 2.0f, 0.2f);
            //var doorInertia = doorShape.CalculateLocalInertia(10.0f);
            //var start = Matrix.Translation(-2.0f, 3.0f, 0);
            //doorBody = PhysicsHandler.CreateBody(10.0f, start, doorShape, doorInertia);
            //doorBody.ActivationState = ActivationState.DisableDeactivation;

            //var pivotA = new BulletSharp.Math.Vector3(-2.0f, 0.0f, 0.0f);
            //var axisA = new BulletSharp.Math.Vector3(0, 1, 0);

            //doorHinge = new HingeConstraint(doorBody, pivotA, axisA);
            //doorHinge.SetLimit(-MathUtil.SIMD_PI * 0.25f, MathUtil.SIMD_PI * 0.25f);
            //PhysicsHandler.World.AddConstraint(doorHinge);

            //doorModel = Primitives.Cube(2, 2, 0.2f, new MeshMaterial
            //{
            //    Diffuse = new Vector4(0, 1, 0, 0.3f)
            //});

            //doorHinge.EnableAngularMotor(true, 10f, 10);

            base.OnLoad();
        }
        #endregion

        #region RENDER
        protected override void OnRenderFrame(FrameEventArgs e)
        {
            if (IsExiting)
                return;

            // TODO: refactor
            // execute all actions, enqueued while loading a model
            int count = Dispatcher.RenderActions.Count;
            if (count > 10)  // clamp amount of executing actions
                count = 10;

            for (int i = 0; i < count; i++)
            {
                Dispatcher.RenderActions.TryDequeue(out Action action);
                action();
            }

            if (Dispatcher.RenderActions.Count == 0)
                Dispatcher.ActionsDone.Set();

            // render main part, i.e. workspace
            RenderCore(e);

            // render GUI
            _imGui.Render(e);

            SwapBuffers();
        }

        private void RenderCore(FrameEventArgs e)
        {
            // workspace viewport
            GL.Viewport((int)(0.25 * Size.X), 0, (int)(0.75 * Size.X), Size.Y);

            GL.Enable(EnableCap.DepthTest);

            // clearing viewport
            GL.Enable(EnableCap.ScissorTest);
            GL.Scissor((int)(0.25 * Size.X), 0, (int)(0.75 * Size.X), Size.Y);
            GL.ClearColor(0.2f, 0.3f, 0.3f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            GL.Disable(EnableCap.ScissorTest);

            ShaderHandler.SetupShaders(Camera);

            RenderCoreOpaque();

            // TODO: render selections, i.e. highlights of the currently selected objects;
            // Tips for GUI:
            // - if there's only one selected object, render a properties tab with that object's properties
            // --- obstacle properties: type, position, orientation and unique properties that define dimensions
            // --- joint properties: coordinate and its range, position and axis (position can be changed directly only for manipulators with straight links)
            // --- links properties are not specified for straight links, because they just connect adjacent joints; for arbitrarily formed links, some future research is needed
            // - if there are multiple objects, only their positions and orientations can be changed (relatively, not absolutely), e.g. with widgets
            // - only a group of common objects can be selected, i.e. obstacles/obstacles, manipulators/manipulators, joints/joints, links/links, etc.

            RenderCoreTransparent();

            RenderCoreIndependent();

            base.OnRenderFrame(e);
        }

        private void RenderCoreOpaque()
        {
            // render the unselected parts of the manipulators
            ManipulatorHandler.RenderUnselected(ShaderHandler.ComplexShader);

            // render the unselected obstacles
            ObstacleHandler.RenderUnselected(ShaderHandler.ComplexShader);

            // render goals
            foreach (var goal in _goalModels)
            {
                if (goal.IsSetup)
                    goal.Render(ShaderHandler.ComplexShader);
            }

            //// render Bezier curve points
            //if (_bezierPoints != null)
            //{
            //    _bezierPoints.Render(ShaderHandler.ComplexShader, () =>
            //    {
            //        GL.PointSize(8);
            //        GL.DrawArrays(PrimitiveType.Points, 0, 4);
            //        GL.PointSize(1);
            //    });
            //}

            // workspace grid
            ObstacleHandler.RenderGrid(ShaderHandler.ComplexShader);
        }

        private void RenderCoreTransparent()
        {
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

            // TODO: all help should be placed in a separate document (documentation)
            // the workspace grid rendering is done lastly, because it's common to render all transparent objects at last
            //
            // the blending function is determined as follows:
            // Color = SourceColor * SourceFactor + DestColor * DestFactor,
            // where
            //     SourceColor - color of the currently rendering fragment,
            //     SourceFactor - its factor,
            //     DestColor - color of the already rendered fragment (the one in the color buffer),
            //     DestFactor - its factor.
            //
            // so, to render transparent floor, we take SourceFactor as source's alpha (floor's alpha) and DestFactor as the remainder of the source's alpha
            // (the visible amount of the opaque object behind the floor)

            //ghostObjectModel.Render(ShaderHandler.ComplexShader);
            //doorModel.Render(ShaderHandler.ComplexShader);

            // render the ground
            ObstacleHandler.RenderGround(ShaderHandler.ComplexShader);

            // render the selected parts of the manipulators
            ManipulatorHandler.RenderSelected(ShaderHandler.ComplexShader);

            // render the selected obstacles
            ObstacleHandler.RenderSelected(ShaderHandler.ComplexShader);

            GL.Disable(EnableCap.Blend);
        }

        private void RenderCoreIndependent()
        {
            GL.Clear(ClearBufferMask.DepthBufferBit);
            InputHandler.Render(ShaderHandler.ComplexShader, null);
        }
        #endregion

        #region UPDATE
        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            if (IsExiting)
                return;

            // update title
            Title = $"{InputHandler.ProjectName} | Framerate: {ImGui.GetIO().Framerate: 0.0} FPS";

            // update handlers states
            InputHandler.Update(e);
            PhysicsHandler.Update(e);
            ManipulatorHandler.Update();
            ObstacleHandler.Update();

            //Console.SetCursorPosition(0, 5);
            //Console.Write("                                                        ");
            //Console.SetCursorPosition(0, 5);
            //Console.WriteLine($"{ObstacleHandler.Obstacles[0].Collider.Body.ActivationState}");

            //if (ManipulatorHandler.Count > 0)
            //{
            //    var res = ObstacleHandler.Obstacles[0].Collider.CollisionPairTest(ObstacleHandler.Obstacles[1].Collider)
            //        /*ManipulatorHandler.Manipulators[0].CollisionTest().Contains(true)*/;

            //    Console.SetCursorPosition(0, 5);
            //    Console.WriteLine("                                                        ");
            //    Console.WriteLine($"Collision is {res}");
            //}

            //if (doorHinge.HingeAngle >= MathUtil.SIMD_PI * 0.2f)
            //    doorHinge.EnableAngularMotor(true, -1f, 1);
            //else if (doorHinge.HingeAngle <= -MathUtil.SIMD_PI * 0.2f)
            //    doorHinge.EnableAngularMotor(true, 1f, 1);

            //var state = doorBody.WorldTransform;
            //OpenTK.Matrix4 stateMatrix = new OpenTK.Matrix4(
            //    state.M11, state.M21, state.M31, state.M41,
            //    state.M12, state.M22, state.M32, state.M42,
            //    state.M13, state.M23, state.M33, state.M43,
            //    state.M14, state.M24, state.M34, state.M44);
            //doorModel.State = stateMatrix;

            //var state = ghostObject.WorldTransform;
            //OpenTK.Matrix4 stateMatrix = new OpenTK.Matrix4(
            //    state.M11, state.M21, state.M31, state.M41,
            //    state.M12, state.M22, state.M32, state.M42,
            //    state.M13, state.M23, state.M33, state.M43,
            //    state.M14, state.M24, state.M34, state.M44);
            //ghostObjectModel.State = stateMatrix;

            //// check for ghost collision
            //Console.SetCursorPosition(0, 10);
            //Console.WriteLine($"Ghost overlaps: {ghostObject.NumOverlappingObjects}");

            //Console.SetCursorPosition(0, 10);
            //if (copy == null && ManipulatorHandler.Count > 0)
            //{
            //    copy = ManipulatorHandler.Manipulators[0].DeepCopy();
            //}

            //if (copy != null)
            //{
            //    copy.CollisionTestGhost().ToArray();
            //}

            //float dt;
            //if (forward)
            //{
            //    dt = (float)e.Time;
            //    if (time > 3)
            //        forward = false;
            //}
            //else
            //{
            //    dt = -(float)e.Time;
            //    if (time < -3)
            //        forward = true;
            //}
            //time += dt;

            //ghostObject.WorldTransform = Matrix.Translation(time * BulletSharp.Math.Vector3.UnitZ);

            //var monitoredBody = (RigidBody)PhysicsHandler.World.CollisionObjectArray[0];
            //object context = "context";
            //PhysicsHandler.World.ContactPairTest(
            //        PhysicsHandler.World.CollisionObjectArray[1],
            //        PhysicsHandler.World.CollisionObjectArray[0],
            //        new CollisionCallback(monitoredBody, context));

            // process physics
            //foreach (RigidBody body in PhysicsHandler.World.CollisionObjectArray)
            //{
            //    if (!"Ground".Equals(body.UserObject) &&
            //        (body.WorldArrayIndex == 1 || body.WorldArrayIndex == 2 || body.WorldArrayIndex == 3))
            //        Cubes[body.UserIndex].State = Convert(body.WorldTransform);
            //}

            // check to see if the window is focused
            //Console.WriteLine($"Focused: {IsFocused}");
            if (!IsFocused)  // TODO: this may cause weird things when window is minimized; check
            {
                return;
            }

            switch (InputHandler.InteractionMode)
            {
                case InteractionMode.Design:

                    // TODO: goals logic should be moved to ManipulatorHandler
                    // update goals positions
                    for (int i = 0; i < ManipulatorHandler.Count; i++)
                    {
                        _goalModels[i].State = Matrix4.CreateTranslation(ManipulatorHandler.Manipulators[i].Goal.ToOpenTK3());

                        foreach (var joint in ManipulatorHandler.Manipulators[i].Joints)  // TODO: for debug use only
                        {
                            if (joint.Active)
                                joint.InitialCoordinate += 0.016f;
                        }
                    }

                    break;
                case InteractionMode.Animate:

                    for (int i = 0; i < ManipulatorHandler.Count; i++)
                    {
                        Manipulator manipulator = ManipulatorHandler.Manipulators[i];

                        // manipulator's path
                        if (manipulator.Path != null)
                        {
                            // move along the path
                            manipulator.FollowPath();

                            // update path model state
                            manipulator.Path.Model.Update();
                        }

                        // random tree
                        if (manipulator.Controller.PathPlanner is RRT rrt)
                        {
                            // update tree model state
                            if (rrt.Tree != null)
                                rrt.Tree.Model.Update();
                        }
                        else if (manipulator.Controller.PathPlanner is GeneticAlgorithm geneticAlgorithm)
                        {
                            if (geneticAlgorithm.Dominant != null && geneticAlgorithm.Dominant.Path != null && geneticAlgorithm.Changed)
                            {
                                //geneticAlgorithm.Dominant.Path.Model.Reset();

                                geneticAlgorithm.Locked = true;

                                geneticAlgorithm.Dominant.BezierCurve.SetModel();

                                //geneticAlgorithm.Dominant.Path.Model.Update();

                                //_bezierPoints = new Model(geneticAlgorithm.Dominant.BezierCurve.Points.Select(point => new MeshVertex
                                //{
                                //    Position = point.ToOpenTK()
                                //}).ToArray(), material: MeshMaterial.Red);

                                geneticAlgorithm.Locked = geneticAlgorithm.Changed = false;
                            }
                        }
                    }

                    break;
            }

            base.OnUpdateFrame(e);
        }

        public void CreateDefaultManipulator()
        {
            var manipulator = ManipulatorHandler.CreateDefaultManipulator();

            // create new model for the manipulator goal
            _goalModels.Add(Primitives.Sphere(0.05f, 5, 5, MeshMaterial.Yellow, Matrix4.Transpose(Matrix4.CreateTranslation(manipulator.Goal.ToOpenTK3()))));
        }
        #endregion

        #region INPUT
        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            base.OnMouseMove(e);
        }
        
        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            _imGui.Scroll(e.Offset);

            base.OnMouseWheel(e);
        }

        protected override void OnTextInput(TextInputEventArgs e)
        {
            _imGui.PressChar(e.AsString[0]);  // TODO: check

            base.OnTextInput(e);
        }

        protected override void OnKeyDown(KeyboardKeyEventArgs e)
        {
            //if (e.Key == Keys.H)
            //    enter = true;

            base.OnKeyDown(e);
        }

        protected override void OnResize(ResizeEventArgs e)
        {
            base.OnResize(e);
        }

        protected override void OnFocusedChanged(FocusedChangedEventArgs e)
        {
            base.OnFocusedChanged(e);
        }
        #endregion

        #region UNLOAD
        protected override void OnUnload()
        {
            // TODO: threads are not disposed correctly; fix!

            // dispose of all the handlers
            InputHandler.Dispose();
            ShaderHandler.Dispose();
            PhysicsHandler.Dispose();
            ManipulatorHandler.Dispose();
            ObstacleHandler.Dispose();            
            
            _imGui.Dispose();

            // remove goals models
            foreach (var goal in _goalModels)
            {
                goal.Dispose();
            }

            // free buffers and program
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);
            GL.BindVertexArray(0);
            GL.UseProgram(0);

            base.OnUnload();
        }
        #endregion
    }
}