using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.Threading.Tasks;

using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Input;

using ImGuiNET;
using Logic;

using Matrix4 = Logic.Matrix4;
using Vector3 = OpenTK.Vector3;
using Vector4 = OpenTK.Vector4;
using Physics;
using BulletSharp;
using System.Threading;
using MathNet.Numerics;
using BulletSharp.Math;
using System.Runtime.InteropServices;
using Logic.PathPlanning;
using Logic.InverseKinematics;

namespace Graphics
{
    public enum InteractionMode
    {
        Design,
        Animate,
        ToDesign,
        ToAnimate
    }

    public class MainWindow : GameWindow
    {
        private static Camera _camera;

        private static readonly List<Model> _goalModels = new List<Model>();  // TODO: consider distributing to the appropriate classes and checking whether the types (Model, etc.) are available at runtime
        private static readonly List<TreeModel> _treeModels = new List<TreeModel>();
        private static readonly List<PathModel> _pathModels = new List<PathModel>();
        private static readonly List<PathModel> _gaModels = new List<PathModel>();
        private static Model _bezierPoints;

        private static float time = 0;
        private static bool forward;

        private static Collider[] _dummyColliders;
        private static Thread[] _dummyTasks;

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
        private static MainWindowImGui _imGui;

        public static Thread MainThread { get; } = Thread.CurrentThread;  // TODO: move to Dispatcher?
        public static InteractionMode Mode { get; private set; } = InteractionMode.Design;

        public MainWindow(int width, int height, GraphicsMode gMode, string title) : 
            base(width, height, gMode, title, GameWindowFlags.Default, DisplayDevice.Default, 4, 6, GraphicsContextFlags.ForwardCompatible) 
        {
            
        }

        #region LOAD
        protected override void OnLoad(EventArgs e)
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

            ShaderHandler.InitializeShaders();

            // attach ImGUI to this window
            _imGui = new MainWindowImGui(this);

            // Camera is 6 units back and has the proper aspect ratio
            _camera = new Camera((float)(0.75 * Width / Height), new Vector3(-5, 3, 5), -15, -45);

            InputHandler.TranslationalWidget = new TranslationalWidget(Vector3.Zero, new (Vector3, Vector4)[3]
            {
                (new Vector3(0.3f, 0, 0), new Vector4(1, 0, 0, 1)),
                (new Vector3(0, 0.3f, 0), new Vector4(0, 1, 0, 1)),
                (new Vector3(0, 0, 0.3f), new Vector4(0, 0, 1, 1))
            });

            // subscribe to the events
            InputHandler.SelectedObjectChanged += OnSelectedObjectChanged;

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

            ObstacleHandler.Add(new Obstacle(Primitives.Sphere(0.5f, 100, 100, new MeshMaterial
            {
                Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
                Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
                Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
                Shininess = 8
            }), PhysicsHandler.CreateKinematicCollider(new SphereShape(0.5f), Matrix.Translation(0, 3, -1.5f))));

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

            base.OnLoad(e);
        }
        #endregion

        #region RENDER
        protected override void OnRenderFrame(FrameEventArgs e)
        {
            // render main part, i.e. workspace
            RenderCore(e);

            // render GUI
            _imGui.Render(e);

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

            SwapBuffers();
        }

        private void RenderCore(FrameEventArgs e)
        {
            // workspace viewport
            GL.Viewport((int)(0.25 * Width), 0, (int)(0.75 * Width), Height);

            GL.Enable(EnableCap.DepthTest);

            // clearing viewport
            GL.Enable(EnableCap.ScissorTest);
            GL.Scissor((int)(0.25 * Width), 0, (int)(0.75 * Width), Height);
            GL.ClearColor(0.2f, 0.3f, 0.3f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            GL.Disable(EnableCap.ScissorTest);

            ShaderHandler.SetupShaders(_camera);

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

            // render paths
            foreach (var path in _pathModels)
            {
                if (path.IsSetup)
                    path.Render(ShaderHandler.ComplexShader);
            }

            // render RRT trees
            foreach (var tree in _treeModels)
            {
                if (tree.IsSetup)
                    tree.Render(ShaderHandler.ComplexShader);
            }

            // render genetic algorithm paths
            foreach (var path in _gaModels)
            {
                if (path.IsSetup)
                    path.Render(ShaderHandler.ComplexShader);
            }

            // render Bezier curve points
            if (_bezierPoints != null)
            {
                _bezierPoints.Render(ShaderHandler.ComplexShader, () =>
                {
                    GL.PointSize(8);
                    GL.DrawArrays(PrimitiveType.Points, 0, 4);
                    GL.PointSize(1);
                });
            }

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
            InputHandler.TranslationalWidget.Render(ShaderHandler.ComplexShader, null);
        }
        #endregion

        #region UPDATE
        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            // update physics controller
            PhysicsHandler.Update((float)e.Time);
            //Console.SetCursorPosition(0, 5);
            //Console.WriteLine("                                                        ");
            //Console.WriteLine($"Num collision objects: {PhysicsHandler.World.NumCollisionObjects}");

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
            if (!Focused)  // TODO: this may cause weird things when window is minimized; check
            {
                return;
            }

            // update camera state
            _camera.UpdateViewMatrix();
            _camera.UpdateProjectionMatrix();

            // process all the input events
            InputHandler.PollEvents(this, _camera, Mouse.GetCursorState(), Keyboard.GetState(), e);

            switch (Mode)
            {
                case InteractionMode.Design:

                    ManipulatorHandler.UpdateDesign();
                    ObstacleHandler.UpdateDesign();

                    // update goals positions
                    for (int i = 0; i < ManipulatorHandler.Count; i++)
                    {
                        _goalModels[i].State = Matrix4.CreateTranslation(ManipulatorHandler.Manipulators[i].Goal);

                        foreach (var joint in ManipulatorHandler.Manipulators[i].Joints)  // TODO: for debug use only
                        {
                            if (joint.Active)
                                joint.Coordinate += 0.016f;
                        }
                    }

                    break;
                case InteractionMode.Animate:

                    //float dt;
                    //if (forward)
                    //{
                    //    dt = (float)e.Time;
                    //    if (time > 1)
                    //        forward = false;
                    //}
                    //else
                    //{
                    //    dt = -(float)e.Time;
                    //    if (time < -1)
                    //        forward = true;
                    //}
                    //time += dt;

                    //if (!ManipHandler.Manipulators.All(x => x.Controller.State == ControllerState.Finished))
                    //{
                    //    ObstacleHandler.Obstacles[0].Move(dt * System.Numerics.Vector3.UnitX);

                    //    var center = ObstacleHandler.Obstacles[0].Collider.Body.CenterOfMassPosition;
                    //    Console.SetCursorPosition(0, 10);
                    //    Console.WriteLine("Center: ({0}, {1}, {2})", center.X, center.Y, center.Z);
                    //    Manager.Obstacles[1].Move(dt * new Vector3(-1, 0, -1));
                    //    Manager.Obstacles[2].Move(-dt * new Vector3(-1, -1, -1));
                    //}

                    for (int i = 0; i < ManipulatorHandler.Count; i++)
                    {
                        Manipulator manipulator = ManipulatorHandler.Manipulators[i];

                        // manipulator's path
                        if (manipulator.Path != null)
                        {
                            // move along the path
                            manipulator.FollowPath();

                            // update path model state
                            _pathModels[i].Update(i);
                        }

                        // random tree
                        if (manipulator.Controller.PathPlanner is RRT rrt)
                        {
                            // update tree model state
                            _treeModels[i].Update(rrt.Tree);
                        }

                        if (GeneticAlgorithm.Dominant.Item2 != null && GeneticAlgorithm.Changed)
                        {
                            _gaModels[i].Reset();

                            GeneticAlgorithm.Locked = true;

                            //var toAdd = GeneticAlgorithm.Dominant.Item2.AddBuffer.DequeueAll().ToList();
                            //var toRemove = GeneticAlgorithm.Dominant.Item2.DelBuffer.DequeueAll().ToList();

                            //_gaModels[i].AddNodes(toAdd);
                            //_gaModels[i].RemoveNodes(toRemove);

                            _bezierPoints = new Model(GeneticAlgorithm.Dominant.Item1.Points.Select(point => new MeshVertex
                            {
                                Position = point.ToOpenTK()
                            }).ToArray(), material: MeshMaterial.Red);

                            GeneticAlgorithm.Locked = GeneticAlgorithm.Changed = false;
                        }
                    }

                    break;
                case InteractionMode.ToDesign:

                    ManipulatorHandler.ToDesign();
                    ObstacleHandler.ToDesign();
                    InputHandler.ToDesign();

                    // update workspace
                    ResetScene();

                    Mode = InteractionMode.Design;

                    break;
                case InteractionMode.ToAnimate:

                    ManipulatorHandler.ToAnimate();
                    ObstacleHandler.ToAnimate();
                    InputHandler.ToAnimate();

                    Mode = InteractionMode.Animate;

                    break;
            }

            // update all models of all the objects
            ManipulatorHandler.UpdateModel();
            ObstacleHandler.UpdateModel();

            base.OnUpdateFrame(e);
        }

        protected void ResetScene()
        {
            // reset trees
            foreach (var tree in _treeModels)
            {
                tree.Reset();
            }

            // reset paths
            foreach (var path in _pathModels)
            {
                path.Reset();
            }
        }

        public static void CreateDefaultManipulator(int linksNumber)
        {
            var manipulator = ManipulatorHandler.CreateDefaultManipulator(linksNumber);

            // create new models for the manipulator goal, path and tree
            _goalModels.Add(Primitives.Sphere(0.05f, 5, 5, MeshMaterial.Yellow, Matrix4.CreateTranslation(manipulator.Goal)));
            _treeModels.Add(new TreeModel(10001, MeshMaterial.Black));
            _pathModels.Add(new PathModel(10001, MeshMaterial.Red));
            _gaModels.Add(new PathModel(10001, MeshMaterial.Black));
        }

        public static void SwitchMode()
        {
            switch (Mode)
            {
                case InteractionMode.Design:
                    Mode = InteractionMode.ToAnimate;
                    break;
                case InteractionMode.Animate:
                    Mode = InteractionMode.ToDesign;
                    break;
            }
                
        }
        #endregion

        #region INPUT
        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            base.OnMouseMove(e);
        }
        
        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            // apply zoom only if no GUI window is currently hovered over
            if (!ImGui.IsWindowHovered(ImGuiHoveredFlags.AnyWindow))
                _camera.Fov -= e.DeltaPrecise;

            base.OnMouseWheel(e);
        }

        protected override void OnKeyPress(KeyPressEventArgs e)
        {
            _imGui.PressChar(e.KeyChar);

            base.OnKeyPress(e);
        }

        protected override void OnResize(EventArgs e)
        {
            // We need to update the aspect ratio once the window has been resized
            _camera.AspectRatio = (float)(0.75 * Width / Height);

            // report to GUI controller about resizing
            //_imGui.WindowResized(Width, Height);

            base.OnResize(e);
        }

        private void OnSelectedObjectChanged(object sender, EventArgs e)
        {
            // notify ImGui about the change
            _imGui.OnSelectedObjectChanged(sender, e);
        }
        #endregion

        #region UNLOAD
        protected override void OnUnload(EventArgs e)
        {
            // TODO: threads are not disposed correctly; fix!

            // dispose of all the handlers
            ManipulatorHandler.Dispose();
            ObstacleHandler.Dispose();
            PhysicsHandler.Dispose();
            InputHandler.Dispose();
            ShaderHandler.Dispose();
            _imGui.Dispose();

            // remove goals models
            foreach (var goal in _goalModels)
            {
                goal.Dispose();
            }

            // remove trees models
            foreach (var tree in _treeModels)
            {
                tree.Dispose();
            }

            // remove paths models
            foreach (var path in _pathModels)
            {
                path.Dispose();
            }

            // free buffers and program
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);
            GL.BindVertexArray(0);
            GL.UseProgram(0);

            base.OnUnload(e);
        }
        #endregion
    }
}