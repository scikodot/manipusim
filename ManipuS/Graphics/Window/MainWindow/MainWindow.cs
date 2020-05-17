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
        private Camera _camera;

        private readonly List<Model> _goalModels = new List<Model>();
        private readonly List<TreeModel> _treeModels = new List<TreeModel>();
        private readonly List<PathModel> _pathModels = new List<PathModel>();
        private readonly List<PathModel> _gaModels = new List<PathModel>();

        private static float time = 0;
        private static bool forward;

        private static GhostObject ghostObject;
        private static Model ghostObjectModel;
        private static GhostPairCallback ghostCallback;
        private static Manipulator copy;
        private static HingeConstraint hinge;

        private static RigidBody doorBody;
        private static Model doorModel;
        private static HingeConstraint doorHinge;

        //Model Crytek;

        // ImGUI variables
        private static object selectedObject;
        private static bool swapPropertiesWindows;
        private static ImGuiTreeNodeFlags _baseTreeNodeFlags = ImGuiTreeNodeFlags.OpenOnArrow;
        private static ImGuiTreeNodeFlags _baseTreeLeafFlags = ImGuiTreeNodeFlags.Leaf | ImGuiTreeNodeFlags.NoTreePushOnOpen;

        public static Thread MainThread { get; } = Thread.CurrentThread;  // TODO: move to Dispatcher?
        public static InteractionMode Mode { get; private set; } = InteractionMode.Design;

        public MainWindow(int width, int height, GraphicsMode gMode, string title) : 
            base(width, height, gMode, title, GameWindowFlags.Default, DisplayDevice.Default, 4, 6, GraphicsContextFlags.ForwardCompatible) 
        {
            
        }

        protected override void OnLoad(EventArgs e)
        {
            //var unptr = Assimp.Unmanaged.AssimpLibrary.Instance.ImportFile(JointPath, Assimp.PostProcessSteps.None, Assimp.Unmanaged.AssimpLibrary.Instance.CreatePropertyStore());
            //var manptr = Assimp.Scene.FromUnmanagedScene(unptr);
            //var ptr = Assimp.Unmanaged.AssimpLibrary.Instance.ApplyPostProcessing(unptr, Assimp.PostProcessSteps.Triangulate);
            //manptr = Assimp.Scene.FromUnmanagedScene(ptr);

            Path.Node node1 = new Path.Node(null, new System.Numerics.Vector3[] { new System.Numerics.Vector3(0, 1, 2) }, null);
            Path.Node node2 = new Path.Node(null, new System.Numerics.Vector3[] { new System.Numerics.Vector3(3, 4, 5) }, null);
            Path.Node node3 = new Path.Node(null, new System.Numerics.Vector3[] { new System.Numerics.Vector3(6, 7, 8) }, null);
            HashSet<Path.Node> path = new HashSet<Path.Node>() { node1, node2, node3 };

            HashSet<Path.Node> path2 = new HashSet<Path.Node>(path);

            node2.Points = new System.Numerics.Vector3[] { new System.Numerics.Vector3(9, 10, 11) };

            ManipulatorHandler.LoadDefaultModels();

            ShaderHandler.InitializeShaders();

            // attach ImGUI to this window
            ImGuiHandler.AttachWindow(this);

            // Camera is 6 units back and has the proper aspect ratio
            _camera = new Camera((float)(0.75 * Width / Height), new Vector3(-5, 3, 5), -15, -45);

            InputHandler.TranslationalWidget = new TranslationalWidget(Vector3.Zero, new (Vector3, Vector4)[3]
            {
                (new Vector3(0.3f, 0, 0), new Vector4(1, 0, 0, 1)),
                (new Vector3(0, 0.3f, 0), new Vector4(0, 1, 0, 1)),
                (new Vector3(0, 0, 0.3f), new Vector4(0, 0, 1, 1))
            });

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

            ObstacleHandler.Add(new Obstacle(Primitives.Sphere(1, 100, 100, new MeshMaterial
            {
                Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
                Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
                Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
                Shininess = 8
            }), PhysicsHandler.CreateDynamicCollider(new SphereShape(1), 1, Matrix.Translation(0, 3, -1.5f))));

            //ObstacleHandler.Add(new Obstacle(Primitives.Cylinder(0.25f, 1, 1, 50, new MeshMaterial
            //{
            //    Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
            //    Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
            //    Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
            //    Shininess = 8
            //}), PhysicsHandler.CreateDynamicCollider(new CylinderShape(0.25f, 1, 0.25f), 1, Matrix.Translation(0, 15, 0))));

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

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            // render main part, i.e. workspace
            RenderCore(e);

            // render GUI
            RenderGUI(e);

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

        protected void RenderCore(FrameEventArgs e)
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
            InputHandler.TranslationalWidget.Render(ShaderHandler.ComplexShader, () =>
            {
                GL.Disable(EnableCap.DepthTest);
                GL.DrawArrays(PrimitiveType.Lines, 0, 2);
                GL.Enable(EnableCap.DepthTest);
            });
        }

        protected void RenderGUI(FrameEventArgs e)
        {
            // update GUI
            ImGuiHandler.Update(this, (float)e.Time);

            //ImGui.ShowDemoWindow();

            // GUI viewport
            GL.Viewport(0, 0, Width, Height);

            // clear viewport
            GL.Enable(EnableCap.ScissorTest);
            GL.Scissor(0, 0, (int)(0.25 * Width), Height);
            GL.ClearColor(0.3f, 0.3f, 0.3f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            GL.Disable(EnableCap.ScissorTest);

            // render all the necessary windows
            RenderMenu();
            RenderManipulatorsWindow();
            RenderObstaclesWindow();
            //RenderAlgorithmsWindow();
            RenderOptionsWindow();

            if (Mode == InteractionMode.Design)
            {
                RenderPropertiesWindow();
            }

            // render controller and check for errors
            ImGuiHandler.Render();
            Util.CheckGLError("End of frame");
        }

        private void RenderMenu()
        {
            if (ImGui.BeginMainMenuBar())
            {
                if (ImGui.BeginMenu("File"))
                {
                    if (ImGui.BeginMenu("Open..."))
                    {
                        if (ImGui.MenuItem("nanosuit.obj"))
                        {

                        }

                        ImGui.EndMenu();
                    }
                    if (ImGui.MenuItem("Save as..."))
                    {

                    }

                    ImGui.EndMenu();
                }

                ImGui.EndMainMenuBar();
            }
        }

        private void UpdateSelection(object selected)
        {
            InputHandler.ClearSelection();

            if (selected == selectedObject)
            {
                selectedObject = null;
            }
            else
            {
                if (selected is Manipulator manipulator)
                {
                    InputHandler.AddSelection(manipulator.Joints.Select(joint => joint.Collider.Body));
                    InputHandler.AddSelection(manipulator.Links.Select(link => link.Collider.Body));
                }
                else if (selected is Joint joint)
                {
                    InputHandler.AddSelection(joint.Collider.Body);
                }
                else if (selected is Link link)
                {
                    InputHandler.AddSelection(link.Collider.Body);
                }
                else if (selected is Obstacle obstacle)
                {
                    InputHandler.AddSelection(obstacle.Collider.Body);
                }

                selectedObject = selected;
                swapPropertiesWindows = !swapPropertiesWindows;
            }
        }

        private ImGuiTreeNodeFlags GetTreeNodeSelectionFlag(object selected)
        {
            return selected == selectedObject ? ImGuiTreeNodeFlags.Selected : ImGuiTreeNodeFlags.None;
        }

        private void RenderManipulatorsWindow()
        {
            // manipulators window
            if (ImGui.Begin("Manipulators",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize |
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, 20));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height - 20)));

                if (ImGui.Button("Create"))
                {
                    CreateDefaultManipulator();
                }

                ImGui.Separator();

                ImGui.PushStyleVar(ImGuiStyleVar.ChildRounding, 5);
                ImGui.BeginChild("ManipulatorList", new System.Numerics.Vector2(ImGui.GetWindowContentRegionWidth(), 118), true);

                if (ManipulatorHandler.Count != 0)
                {
                    for (int i = 0; i < ManipulatorHandler.Count; i++)
                    {
                        var manipulator = ManipulatorHandler.Manipulators[i];
                        bool manipulatorTreeNodeOpen = ImGui.TreeNodeEx($"Manipulator {i}", _baseTreeNodeFlags | GetTreeNodeSelectionFlag(manipulator));
                        if (ImGui.IsItemDeactivated() && !ImGui.IsItemToggledOpen())
                        {
                            UpdateSelection(manipulator);
                        }

                        if (manipulatorTreeNodeOpen)  // TODO: refactor, perhaps move all updates to UpdateFrame
                        {
                            for (int j = 0; j < manipulator.Links.Length; j++)
                            {
                                var joint = manipulator.Joints[j];
                                ImGui.TreeNodeEx($"Joint {j}", _baseTreeLeafFlags | GetTreeNodeSelectionFlag(joint));
                                if (ImGui.IsItemDeactivated())
                                {
                                    UpdateSelection(joint);
                                }

                                var link = manipulator.Links[j];
                                ImGui.TreeNodeEx($"Link {j}", _baseTreeLeafFlags | GetTreeNodeSelectionFlag(link));
                                if (ImGui.IsItemDeactivated())
                                {
                                    UpdateSelection(link);
                                }
                            }

                            var gripper = manipulator.Joints[manipulator.Joints.Length - 1];
                            ImGui.TreeNodeEx($"Gripper", _baseTreeLeafFlags | GetTreeNodeSelectionFlag(gripper));
                            if (ImGui.IsItemDeactivated())
                            {
                                UpdateSelection(gripper);
                            }

                            ImGui.TreePop();

                            //ImGui.InputInt("Links number", ref MB[j].N);

                            //WorkspaceBuffer.ConfigureArrays(j);
                        }
                    }
                }
                else
                {
                    ImGui.Text("Empty.");
                }

                ImGui.EndChild();

                ImGui.End();
            }
        }

        public void CreateDefaultManipulator()
        {
            var manipulator = ManipulatorHandler.CreateDefaultManipulator(3);

            // create new models for the manipulator goal, path and tree
            _goalModels.Add(Primitives.Sphere(0.05f, 5, 5, MeshMaterial.Yellow, Matrix4.CreateTranslation(manipulator.Goal)));
            _treeModels.Add(new TreeModel(10001, MeshMaterial.Black));
            _pathModels.Add(new PathModel(10001, MeshMaterial.Red));
            _gaModels.Add(new PathModel(10001, MeshMaterial.Black));
        }

        private void RenderObstaclesWindow()
        {
            // obstacles window
            if (ImGui.Begin("Obstacles",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize |
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.25 * Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                if (ImGui.Button("Create"))
                {
                    ImGui.OpenPopup("ObstacleCreate");
                }

                if (ImGui.BeginPopup("ObstacleCreate"))
                {
                    foreach (var shape in ObstacleHandler.ObstacleShapes)
                    {
                        if (ImGui.Selectable(shape))
                        {
                            // TODO: create an obstacle with the specified shape
                        }
                    }

                    ImGui.EndPopup();
                }

                ImGui.Separator();

                ImGui.PushStyleVar(ImGuiStyleVar.ChildRounding, 5);
                ImGui.BeginChild("ObstacleList", new System.Numerics.Vector2(ImGui.GetWindowContentRegionWidth(), 138), true);

                if (ObstacleHandler.Count != 0)
                {
                    for (int i = 0; i < ObstacleHandler.Count; i++)
                    {
                        var obstacle = ObstacleHandler.Obstacles[i];
                        ImGui.TreeNodeEx($"Obstacle {i}", _baseTreeLeafFlags | GetTreeNodeSelectionFlag(obstacle));
                        if (ImGui.IsItemDeactivated())
                        {
                            UpdateSelection(obstacle);
                        }
                    }
                }
                else
                {
                    ImGui.Text("Empty.");
                }

                ImGui.EndChild();

                ImGui.End();
            }
        }

        private void RenderAlgorithmsWindow()
        {
            // algorithm window
            if (ImGui.Begin("Algorithm",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize |
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.5 * Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                ImGui.NewLine();
                ImGui.PushID(0);
                ImGui.Text("Inverse kinematics solver:");
                ImGui.Combo("Type",
                    ref WorkspaceBuffer.InverseKinematicsBuffer.InverseKinematicsSolverID,
                    Logic.InverseKinematics.InverseKinematicsSolver.Types,
                    Logic.InverseKinematics.InverseKinematicsSolver.Types.Length);
                ImGui.InputFloat("Precision", ref WorkspaceBuffer.InverseKinematicsBuffer.Precision);
                ImGui.InputInt("Iterations", ref WorkspaceBuffer.InverseKinematicsBuffer.MaxTime);
                ImGui.InputFloat("Step size (deg)", ref WorkspaceBuffer.InverseKinematicsBuffer.StepSize);

                ImGui.NewLine();
                ImGui.PushID(1);
                ImGui.Text("Path planner:");
                ImGui.Combo("Type",
                    ref WorkspaceBuffer.PathPlanningBuffer.PathPlannerID,
                    Logic.PathPlanning.PathPlanner.Types,
                    Logic.PathPlanning.PathPlanner.Types.Length);
                ImGui.InputInt("Attractors number", ref WorkspaceBuffer.PathPlanningBuffer.AttrNum);
                ImGui.InputInt("Iterations", ref WorkspaceBuffer.PathPlanningBuffer.k);
                ImGui.InputFloat("Step size", ref WorkspaceBuffer.PathPlanningBuffer.d);

                ImGui.End();
            }
        }

        private void RenderOptionsWindow()
        {
            // options and info window
            if (ImGui.Begin("Options & Info",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.75 * Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                string targetMode;
                switch (Mode)
                {
                    case InteractionMode.Animate:
                    case InteractionMode.ToDesign:
                        targetMode = "Design";
                        break;
                    case InteractionMode.Design:
                    case InteractionMode.ToAnimate:
                        targetMode = "Animate";
                        break;
                    default:
                        throw new ArgumentException("The given mode is unsupported!", "MainWindow.Mode");
                }

                if (ImGui.Button(targetMode))
                {
                    if (Mode == InteractionMode.Design)
                        Mode = InteractionMode.ToAnimate;
                    else if (Mode == InteractionMode.Animate)
                        Mode = InteractionMode.ToDesign;
                }
                if (ImGui.IsItemHovered())
                {
                    ImGui.SameLine();
                    ImGui.TextWrapped("Switch between design/animate modes");
                }

                if (ImGui.Button("Screenshot"))
                {
                    // inform the input handler that the window capture has been queried
                    InputHandler.Capture = true;
                }
                if (ImGui.IsItemHovered())
                {
                    ImGui.SameLine();
                    ImGui.TextWrapped("Takes a picture of the entire window");
                }

                // savepath for captured screenshot
                ImGui.InputText("Savepath", ref InputHandler.ScreenshotsPath, 100);
                InputHandler.TextIsEdited = ImGui.IsItemActive();

                // application current framerate
                ImGui.SetCursorScreenPos(new System.Numerics.Vector2(8, Height - 8 - ImGui.CalcTextSize("Framerate:").Y));
                ImGui.Text(string.Format("Framerate: {0:F1} FPS", ImGui.GetIO().Framerate));

                ImGui.End();
            }
        }

        private void RenderPropertiesWindow()
        {
            if (selectedObject is Manipulator manipulator)
            {
                RenderPropertiesWindowTemplate("Manipulator properties", ManipulatorProperties, manipulator);
            }
            else if (selectedObject is Joint joint)
            {
                RenderPropertiesWindowTemplate("Joint properties", JointProperties, joint);
            }
            else if (selectedObject is Link link)
            {
                RenderPropertiesWindowTemplate("Link properties", LinkProperties, link);
            }
            else if (selectedObject is Obstacle obstacle)
            {
                RenderPropertiesWindowTemplate("Obstacle properties", ObstacleProperties, obstacle);
            }
        }

        private void RenderPropertiesWindowTemplate<T>(string title, Action<T> renderProperties, T selectable) where T: ISelectable
        {
            if (ImGui.Begin(title/* + (changed ? "##first" : "##last")*/,
                    ImGuiWindowFlags.NoCollapse |
                    ImGuiWindowFlags.NoMove |
                    ImGuiWindowFlags.NoResize |
                    ImGuiWindowFlags.HorizontalScrollbar))
            {
                // swap window on the ID stack to not inherit fields inputs from the previous objects
                ImGui.PushID(swapPropertiesWindows ? 1 : 0);

                // set position and size of the window
                ImGui.SetWindowPos(new System.Numerics.Vector2((int)(0.25 * Width), (int)(0.75 * Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                // perform the necessary actions
                renderProperties(selectable);

                // remove window's ID from the stack, just in case
                ImGui.PopID();
            }
        }

        private void ManipulatorProperties(Manipulator manipulator)
        {
            ImGui.Text($"Time spent: {manipulator.Controller.Timer.ElapsedMilliseconds / 1000.0f} s");  // TODO: move to Statistics window
            ImGui.Checkbox($"Show collider", ref manipulator.ShowCollider);
            ImGui.InputFloat3("Goal", ref manipulator.Goal);

            ImGui.Separator();

            var IB = WorkspaceBuffer.InverseKinematicsBuffer;
            var PB = WorkspaceBuffer.PathPlanningBuffer;

            int inverseKinematicsType = (int)manipulator.Controller.InverseKinematicsSolverType;
            int prevInverseKinematicsType = inverseKinematicsType;
            ImGui.Text("Inverse kinematics solver:");
            ImGui.PushID(0);
            ImGui.Combo("Type",
                ref inverseKinematicsType,
                InverseKinematicsSolver.Types,
                InverseKinematicsSolver.Types.Length);
            manipulator.Controller.InverseKinematicsSolverType = (InverseKinematicsSolverType)inverseKinematicsType;

            // change inverse kinematics solver if queried
            if (inverseKinematicsType != prevInverseKinematicsType)
            {
                switch (manipulator.Controller.InverseKinematicsSolverType)
                {
                    case InverseKinematicsSolverType.JacobianTranspose:
                        manipulator.Controller.PlanSolver = new JacobianTranspose(IB.Precision, IB.StepSize, IB.MaxTime);
                        break;
                    case InverseKinematicsSolverType.JacobianInverse:
                        manipulator.Controller.PlanSolver = new JacobianInverse(IB.Precision, IB.StepSize, IB.MaxTime);
                        break;
                    case InverseKinematicsSolverType.DampedLeastSquares:
                        manipulator.Controller.PlanSolver = new DampedLeastSquares(IB.Precision, IB.StepSize, IB.MaxTime);
                        break;
                }
            }

            // inverse kinematics solver properties
            ImGui.InputInt("Max time", ref manipulator.Controller.PlanSolver.MaxTime);

            if (manipulator.Controller.PlanSolver is JacobianTranspose jacobianTranspose)
            {
                ImGui.InputFloat("Base damping coefficient", ref jacobianTranspose.Alpha);
            }
            else if (manipulator.Controller.PlanSolver is JacobianInverse jacobianInverse)
            {
                // TODO: input something here?
            }
            else if (manipulator.Controller.PlanSolver is DampedLeastSquares dampedLeastSquares)
            {
                ImGui.InputFloat("Damping coefficient", ref dampedLeastSquares.Lambda);
            }

            ImGui.Separator();

            // TODO: capture type here to compare with the new type
            ImGui.Text("Path planner:");
            ImGui.PushID(1);

            int pathPlannerType = (int)manipulator.Controller.PathPlannerType;
            int prevPathPlannerType = pathPlannerType;
            ImGui.Combo("Type",
                ref pathPlannerType,
                PathPlanner.Types,
                PathPlanner.Types.Length);
            manipulator.Controller.PathPlannerType = (PathPlannerType)pathPlannerType;

            // change path planner if queried
            if (pathPlannerType != prevPathPlannerType)
            {
                switch (manipulator.Controller.PathPlannerType)
                {
                    case PathPlannerType.RRT:
                        manipulator.Controller.PathPlanner = new RRT(PB.k, false, PB.d);
                        break;
                    case PathPlannerType.DynamicRRT:
                        manipulator.Controller.PathPlanner = new DynamicRRT(PB.k, false, PB.d, PB.k / 10);
                        break;
                    case PathPlannerType.GeneticAlgorithm:
                        throw new NotImplementedException("Genetic algorithm planner is not implemented yet!");
                        break;
                }
            }

            // path planner properties
            ImGui.InputInt("Max time", ref manipulator.Controller.PathPlanner.MaxTime);

            if (manipulator.Controller.PathPlanner is RRT rrt)
            {
                ImGui.Text($"Tree size: {(manipulator.Tree == null ? 0 : manipulator.Tree.Count)} nodes");  // TODO: move to Statistics window
                ImGui.Checkbox($"Show tree", ref manipulator.ShowTree);
                ImGui.InputFloat("Step", ref rrt.Step);
                ImGui.InputFloat("Threshold", ref rrt.Threshold);

                if (manipulator.Controller.PathPlanner is DynamicRRT dynamicRRT)  // TODO: add attractors property
                {
                    ImGui.InputInt("Trim period", ref dynamicRRT.TrimPeriod);
                }
            }
            // TODO: add GeneticAlgorithm

            // TODO: switch MotionControl
        }

        private void JointProperties(Joint joint)
        {
            ImGui.Checkbox("Show collider", ref joint.ShowCollider);

            ImGui.InputFloat3("Axis", ref joint.InitialAxis);
            ImGui.InputFloat3("Position", ref joint.InitialPosition);

            if (joint.Collider is SphereCollider sphere)
            {
                ImGui.InputFloat("Radius", ref sphere.Radius);
            }

            ImGui.Separator();

            ImGui.InputFloat("Coordinate", ref joint.InitialCoordinate);
            ImGui.InputFloat2("Coordinate range", ref joint.CoordinateRange);
        }

        private void LinkProperties(Link link)
        {
            if (ImGui.BeginTabBar("LinkTabs"))
            {
                if (ImGui.BeginTabItem("Model"))
                {
                    // TODO: add model specific properties

                    ImGui.EndTabItem();
                }

                if (ImGui.BeginTabItem("Collider"))
                {
                    ImGui.Checkbox("Show collider", ref link.ShowCollider);

                    // TODO: add length property
                    if (link.Collider is CylinderCollider cylinder)
                    {
                        ImGui.InputFloat("Radius", ref cylinder.Radius);
                        ImGui.InputFloat("Half length", ref cylinder.HalfLength, 0, 0, null, ImGuiInputTextFlags.ReadOnly);  // TODO: this should not be read-only; implement!
                    }

                    ImGui.EndTabItem();
                }

                ImGui.EndTabBar();
            }
        }

        private void ObstacleProperties(Obstacle obstacle)
        {
            ImGui.Text($"Shape type: {obstacle.ShapeType}");

            int type = (int)obstacle.Type;
            ImGui.Combo("Physics type", ref type,
                PhysicsHandler.RigidBodyTypes,
                PhysicsHandler.RigidBodyTypes.Length);  // TODO: add mass property to Dynamic bodies!
            obstacle.Type = (RigidBodyType)type;

            ImGui.Checkbox("Show collider", ref obstacle.ShowCollider);
            ImGui.InputFloat3("Orientation", ref obstacle.Orientation);
            ImGui.InputFloat3("Position", ref obstacle.InitialPosition);

            if (obstacle.Collider is BoxCollider box)  // TODO: handle zero cases; when the dimensions are zeroed, objects disappear!!!
            {
                ImGui.InputFloat3("Half extents", ref box.Size);
            }
            else if (obstacle.Collider is SphereCollider sphere)
            {
                ImGui.InputFloat("Radius", ref sphere.Radius);
            }
            else if (obstacle.Collider is CylinderCollider cylinder)
            {
                ImGui.InputFloat("Radius", ref cylinder.Radius);
                ImGui.InputFloat("Half length", ref cylinder.HalfLength);
            }
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            // update physics controller
            PhysicsHandler.Update((float)e.Time);

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

                    AttachWidgets();

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
                        Manipulator manip = ManipulatorHandler.Manipulators[i];
                        if (manip.Tree != null)
                        {
                            Console.SetCursorPosition(0, 10);
                            Console.WriteLine(manip.Tree.Count);
                        }

                        // manipulator's path
                        if (manip.Path != null)
                        {
                            // move along the path
                            manip.FollowPath();

                            // update path model state
                            _pathModels[i].Update(i);
                        }

                        // random tree
                        if (manip.Tree != null)
                        {
                            // update tree model state
                            _treeModels[i].Update(i);
                        }

                        if (GeneticAlgorithm.Dominant != null && GeneticAlgorithm.Changed)
                        {
                            _gaModels[i].Reset();

                            GeneticAlgorithm.Locked = true;

                            var toAdd = GeneticAlgorithm.Dominant.AddBuffer.DequeueAll().ToList();
                            var toRemove = GeneticAlgorithm.Dominant.DelBuffer.DequeueAll().ToList();

                            _gaModels[i].AddNodes(toAdd);
                            _gaModels[i].RemoveNodes(toRemove);

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

        private void AttachWidgets()  // TODO: refactor; move somewhere else? rename?
        {
            if (InputHandler.SelectedObjects.Count == 1)
            {
                var selected = InputHandler.SelectedObjects[0].UserObject;
                if (selected != selectedObject)
                {
                    selectedObject = selected;
                    swapPropertiesWindows = !swapPropertiesWindows;
                }

                // attach the widget
                InputHandler.TranslationalWidget.Attach(selectedObject as ITranslatable);
            }
            else
            {
                if (InputHandler.SelectedObjects.Count == 0 || !(selectedObject is Manipulator))
                    selectedObject = null;

                // detach the widget
                InputHandler.TranslationalWidget.Detach();
            }
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


        protected override void OnResize(EventArgs e)
        {
            // We need to update the aspect ratio once the window has been resized
            _camera.AspectRatio = (float)(0.75 * Width / Height);

            // report to GUI controller about resizing
            ImGuiHandler.WindowResized(Width, Height);

            base.OnResize(e);
        }

        protected override void OnUnload(EventArgs e)
        {
            // TODO: threads are not disposed correctly; fix!

            // dispose of all the handlers
            ManipulatorHandler.Dispose();
            ObstacleHandler.Dispose();
            PhysicsHandler.Dispose();
            InputHandler.Dispose();
            ShaderHandler.Dispose();
            ImGuiHandler.Dispose();

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

        protected override void OnKeyPress(KeyPressEventArgs e)
        {
            ImGuiHandler.PressChar(e.KeyChar);

            base.OnKeyPress(e);            
        }
    }
}