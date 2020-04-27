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
using System.Drawing;

namespace Graphics
{
    public class Window : GameWindow
    {
        // main graphics objects
        private static Shader _shader;
        private ImGuiController controller;
        private Camera _camera;

        // workspace grid
        private float[] transparencyMask =
        {
            // mask used to make a floor half-transparent
            10.0f, 0.0f, 10.0f,     1.0f, 1.0f, 1.0f, 0.5f,
            -10.0f, 0.0f, 10.0f,    1.0f, 1.0f, 1.0f, 0.5f,
            -10.0f, 0.0f, -10.0f,   1.0f, 1.0f, 1.0f, 0.5f,
            10.0f, 0.0f, -10.0f,    1.0f, 1.0f, 1.0f, 0.5f
        };
        private float[] gridLines =
        {
            // X axis lines
            10.0f, 0.0f, 0.0f,      1.0f, 1.0f, 1.0f, 1.0f,
            -10.0f, 0.0f, 0.0f,     1.0f, 1.0f, 1.0f, 1.0f,

            // Y axis lines (Z in GL format)
            0.0f, 0.0f, 10.0f,      1.0f, 1.0f, 1.0f, 1.0f,
            0.0f, 0.0f, -10.0f,     1.0f, 1.0f, 1.0f, 1.0f
        };

        // all the needed entities
        PlainModel grid, gridFloor;
        PlainModel[] goal, path;

        HashSet<Logic.PathPlanning.Tree.Node>[] tree;

        // variables for mouse state processing
        private bool _firstMove = true;
        private Vector2 _lastPos;

        // misc variables
        private bool Capture = false;
        private static System.IO.DirectoryInfo projDir = System.IO.Directory.GetParent(Environment.CurrentDirectory).Parent;
        private static System.IO.DirectoryInfo solDir = projDir.Parent;

        private static string ProjectDirectory = projDir.FullName;
        private static string SolutionDirectory = solDir.FullName;

        private static string VertexShader = ProjectDirectory + @"\Graphics\Shader\Shaders\VertexShader.glsl";
        private static string FragmentShader = ProjectDirectory + @"\Graphics\Shader\Shaders\FragmentShader.glsl";
        private static string LineShader = ProjectDirectory + @"\Graphics\Shader\Shaders\LineShader.glsl";

        private static string SavePath = SolutionDirectory + @"\Screenshots";
        private static string NanosuitPath = SolutionDirectory + @"\Resources\Models\nanosuit\nanosuit.obj";
        private static string LinkPath = SolutionDirectory + @"\Resources\Models\manipulator\Link.obj";
        private static string JointPath = SolutionDirectory + @"\Resources\Models\manipulator\Joint.obj";
        private static string GripperPath = SolutionDirectory + @"\Resources\Models\manipulator\Gripper.obj";

        public static float time = 0;
        public static bool forward;
        private bool ManipLoaded = false;
        private bool TextEdited = false;

        // 3D model
        ComplexModel Crytek;
        public static Shader lineShader;
        public static PlainModel pointMoveable;
        public static Vector2 pointScreen;

        public static AxesWidget widget;
        public static bool IsAxisX;

        public Window(int width, int height, GraphicsMode gMode, string title) : 
            base(width, height, gMode, title, GameWindowFlags.Default, DisplayDevice.Default, 4, 6, GraphicsContextFlags.ForwardCompatible) { }

        protected override void OnLoad(EventArgs e)
        {
            //var unptr = Assimp.Unmanaged.AssimpLibrary.Instance.ImportFile(JointPath, Assimp.PostProcessSteps.None, Assimp.Unmanaged.AssimpLibrary.Instance.CreatePropertyStore());
            //var manptr = Assimp.Scene.FromUnmanagedScene(unptr);
            //var ptr = Assimp.Unmanaged.AssimpLibrary.Instance.ApplyPostProcessing(unptr, Assimp.PostProcessSteps.Triangulate);
            //manptr = Assimp.Scene.FromUnmanagedScene(ptr);

            // defining ImGui controller
            controller = new ImGuiController(Width, Height);
            
            // retrieving shaders' files
            _shader = new Shader(VertexShader, FragmentShader);

            lineShader = new Shader(VertexShader, LineShader);

            // Camera is 6 units back and has the proper aspect ratio
            _camera = new Camera(Vector3.UnitZ * 6, (float)(0.75 * Width / Height));

            // workspace grid
            grid = new PlainModel(lineShader, gridLines);
            gridFloor = new PlainModel(lineShader, transparencyMask, new uint[] { 1, 0, 3, 1, 2, 3, 1 });

            pointMoveable = new PlainModel(lineShader, new float[] { 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f });

            widget = new AxesWidget();
            widget.Attach(pointMoveable);

            base.OnLoad(e);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            controller.Update(this, (float)e.Time);

            // render main part, i.e. workspace
            RenderCore(e);

            // render GUI
            RenderGUI();

            // capture screen after render if queried
            if (Capture)
            {
                Utils.ScreenCapture(this, SavePath);
                Capture = false;
            }

            // execute all actions, enqueued while loading a model
            int count = Dispatcher.RenderActions.Count;
            if (count > 10)  // clamp amount of executing actions to get rid of microfreezes
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

            GL.Enable(EnableCap.DepthTest);  // TODO: fix depth test so that it doesn't hide objects behind alpha-fragments

            // clearing viewport
            GL.Enable(EnableCap.ScissorTest);
            GL.Scissor((int)(0.25 * Width), 0, (int)(0.75 * Width), Height);
            GL.ClearColor(0.2f, 0.3f, 0.3f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            GL.Disable(EnableCap.ScissorTest);

            // attaching shader
            _shader.Use();

            // set view and projection matrices;
            // these matrices come pre-transposed, so there's no need to transpose them again (see VertexShader file)
            _shader.SetMatrix4("view", ref _camera.ViewMatrix, false);
            _shader.SetMatrix4("projection", ref _camera.ProjectionMatrix, false);

            // set general properties
            _shader.SetVector3("viewPos", _camera.Position);

            // set directional light properties
            _shader.SetVector3("dirLight[0].direction", new Vector3(1.0f, 0.0f, 0.0f));
            _shader.SetVector3("dirLight[1].direction", new Vector3(0.0f, -1.0f, 0.0f));
            _shader.SetVector3("dirLight[2].direction", new Vector3(0.0f, 0.0f, -1.0f));
            for (int i = 0; i < 3; i++)
            {
                _shader.SetVector3($"dirLight[{i}].ambient", new Vector3(0.05f, 0.05f, 0.05f));
                _shader.SetVector3($"dirLight[{i}].diffuse", new Vector3(0.75f, 0.75f, 0.75f));
                _shader.SetVector3($"dirLight[{i}].specular", new Vector3(0.5f, 0.5f, 0.5f));
            }

            // setup line shader
            lineShader.Use();
            lineShader.SetMatrix4("view", ref _camera.ViewMatrix, false);
            lineShader.SetMatrix4("projection", ref _camera.ProjectionMatrix, false);
            lineShader.SetVector3("color", Vector3.One);

            pointMoveable.Render(() =>
            {
                GL.PointSize(20);
                GL.DrawArrays(PrimitiveType.Points, 0, 1);
                GL.PointSize(1);
            });

            widget.Render(() =>
            {
                GL.DrawArrays(PrimitiveType.Lines, 0, 2);
            });

            if (ManipLoaded)
            {
                // render obstacles
                foreach (var obstacle in Manager.Obstacles)
                {
                    obstacle.Render(lineShader, true);
                }

                for (int i = 0; i < Manager.Manipulators.Length; i++)
                {
                    Manipulator manip = Manager.Manipulators[i];

                    // render manipulator
                    manip.Render(_shader);

                    // render goal
                    if (goal[i] != default)
                    {
                        goal[i].Render(() =>
                        {
                            GL.PointSize(5);
                            GL.DrawArrays(PrimitiveType.Points, 0, 1);
                            GL.PointSize(1);
                            GL.DrawArrays(PrimitiveType.Points, 1, 100);
                        });
                    }

                    // render path
                    if (manip.Path != null)
                    {
                        int count = manip.Path.Count;
                        path[i].Render(() =>
                        {
                            GL.DrawArrays(PrimitiveType.LineStrip, 0, count);
                        });
                    }

                    // render tree
                    if (WorkspaceBuffer.ManipBuffer[i].ShowTree)
                    {
                        foreach (var node in tree[i])  // TODO: node can become null; reason - Null in Add/Del buffers; inspect why!
                                                       // this may be a problem with either thread communication, or queue extension,
                                                       // because internally in tree no Null appears (tested on a single manipulator)
                        {
                            if (node.Model == default)
                                node.Model = CreateTreeBranch(node.Point, node.Parent.Point);

                            node.Model.Render(() =>
                            {
                                GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                            });
                        }
                    }
                }
            }

            // workspace grid
            grid.State = Matrix4.Identity;
            grid.Render(() =>
            {
                GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                GL.DrawArrays(PrimitiveType.LineStrip, 2, 2);
            });
            for (int i = 1; i < 11; i++)
            {
                grid.State = Matrix4.CreateTranslation(System.Numerics.Vector3.UnitZ * i);
                grid.Render(() => GL.DrawArrays(PrimitiveType.LineStrip, 0, 2));
                grid.State = Matrix4.CreateTranslation(System.Numerics.Vector3.UnitZ * -i);
                grid.Render(() => GL.DrawArrays(PrimitiveType.LineStrip, 0, 2));

                grid.State = Matrix4.CreateTranslation(System.Numerics.Vector3.UnitX * i);
                grid.Render(() => GL.DrawArrays(PrimitiveType.LineStrip, 2, 2));
                grid.State = Matrix4.CreateTranslation(System.Numerics.Vector3.UnitX * -i);
                grid.Render(() => GL.DrawArrays(PrimitiveType.LineStrip, 2, 2));
            }

            gridFloor.Render(() =>  // TODO: all help should be placed in a separate document (aka documentation)
            {
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
                GL.Enable(EnableCap.Blend);
                GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

                GL.DrawElements(BeginMode.Triangles, 7, DrawElementsType.UnsignedInt, 0);

                GL.Disable(EnableCap.Blend);
            });

            //ImGui.ShowDemoWindow();

            base.OnRenderFrame(e);
        }

        protected void RenderGUI()
        {
            // GUI viewport
            GL.Viewport(0, 0, Width, Height);

            // clearing viewport
            GL.Enable(EnableCap.ScissorTest);
            GL.Scissor(0, 0, (int)(0.25 * Width), Height);
            GL.ClearColor(0.3f, 0.3f, 0.3f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            GL.Disable(EnableCap.ScissorTest);

            if (ImGui.BeginMainMenuBar())
            {
                if (ImGui.BeginMenu("File"))
                {
                    if (ImGui.BeginMenu("Open..."))
                    {
                        if (ImGui.MenuItem("nanosuit.obj"))
                        {
                            Dispatcher.ActiveTasks.Add(Task.Run(() =>
                            {
                                // load components' models
                                var jointModel = new ComplexModel(JointPath);
                                var linkModel = new ComplexModel(LinkPath);
                                var gripperModel = new ComplexModel(GripperPath);

                                var MB = WorkspaceBuffer.ManipBuffer;
                                for (int i = 0; i < MB.Length; i++)
                                {
                                    for (int j = 0; j < MB[i].N; j++)
                                    {
                                        MB[i].Links[j].Model = linkModel;
                                        MB[i].Joints[j].Model = jointModel;
                                    }

                                    MB[i].Joints[MB[i].N].Model = gripperModel;
                                }

                                // wait for loading process to finish
                                Dispatcher.ActionsDone.Reset();
                                Dispatcher.ActionsDone.WaitOne();

                                // update workspace with newly loaded model
                                UpdateWorkspace();
                                Dispatcher.UpdateThreads();
                                ManipLoaded = true;
                                //Crytek = new Model(ManipPath);

                                Dispatcher.RunObstacles();
                            }));
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

            // manipulators window
            if (ImGui.Begin("Manipulators",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize |
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, 20));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height - 20)));

                if (Manager.Manipulators != null)
                {
                    var MB = WorkspaceBuffer.ManipBuffer;
                    for (int j = 0; j < Manager.Manipulators.Length; j++)
                    {
                        if (ImGui.TreeNode($"Manip {j}"))
                        {
                            ImGui.Text($"Time spent: {Dispatcher.timers[j].ElapsedMilliseconds / 1000.0f} s");

                            int count = Manager.Manipulators[j].Tree == null ? 0 : Manager.Manipulators[j].Tree.Count;
                            ImGui.Checkbox($"Show tree ({count} verts)", ref MB[j].ShowTree);
                            ImGui.InputFloat3("Goal", ref MB[j].Goal);
                            ImGui.InputInt("Links number", ref MB[j].N);

                            WorkspaceBuffer.ConfigureArrays(j);

                            if (ImGui.TreeNode("Links"))
                            {
                                for (int i = 0; i < MB[j].N; i++)
                                {
                                    ImGui.InputFloat($"Link {i}", ref MB[j].Links[i].Length);
                                }
                                ImGui.TreePop();
                            }

                            if (ImGui.TreeNode("Joints"))
                            {
                                for (int i = 0; i < MB[j].N + 1; i++)
                                {
                                    if (ImGui.TreeNode(i == 0 ? "Base" : $"Joint {i}"))
                                    {
                                        ImGui.InputFloat3("Axis", ref WorkspaceBuffer.JointAxes[j][i]);
                                        ImGui.InputFloat3("Position", ref WorkspaceBuffer.JointPositions[j][i]);
                                        ImGui.InputFloat("Initial GC (deg)", ref MB[j].Joints[i].q);
                                        ImGui.InputFloat2("GC range", ref MB[j].Joints[i].qRanges);
                                        ImGui.TreePop();
                                    }
                                }
                                ImGui.TreePop();
                            }
                            ImGui.TreePop();
                        }
                    }
                }
                else
                {
                    ImGui.Text("No manipulators at the scene.");
                }
                ImGui.End();
            }

            // obstacles window
            if (ImGui.Begin("Obstacles",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize |
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.25 * Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                if (Manager.Obstacles != null)
                {
                    for (int i = 0; i < Manager.Obstacles.Length; i++)
                    {
                        if (ImGui.TreeNode($"Obst {i}"))
                        {
                            ImGui.Checkbox("Show collider", ref WorkspaceBuffer.ObstBuffer[i].ShowCollider);
                            ImGui.InputFloat("Radius", ref WorkspaceBuffer.ObstBuffer[i].Radius);
                            ImGui.InputFloat3("Center", ref WorkspaceBuffer.ObstBuffer[i].Center);
                            ImGui.InputInt("Points number", ref WorkspaceBuffer.ObstBuffer[i].PointsNum);
                            ImGui.TreePop();
                        }
                    }
                }
                else
                {
                    ImGui.Text("No obstacles at the scene.");
                }
                ImGui.End();
            }


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
                    ref WorkspaceBuffer.AlgBuffer.InverseKinematicsSolverID,
                    Logic.InverseKinematics.IKSolver.Types,
                    Logic.InverseKinematics.IKSolver.Types.Length);
                ImGui.InputFloat("Precision", ref WorkspaceBuffer.AlgBuffer.Precision);
                ImGui.InputInt("Iterations", ref WorkspaceBuffer.AlgBuffer.MaxTime);
                ImGui.InputFloat("Step size (deg)", ref WorkspaceBuffer.AlgBuffer.StepSize);

                ImGui.NewLine();
                ImGui.PushID(1);
                ImGui.Text("Path planner:");
                ImGui.Combo("Type",
                    ref WorkspaceBuffer.AlgBuffer.PathPlannerID,
                    Logic.PathPlanning.PathPlanner.Types,
                    Logic.PathPlanning.PathPlanner.Types.Length);
                ImGui.InputInt("Attractors number", ref WorkspaceBuffer.AlgBuffer.AttrNum);
                ImGui.InputInt("Iterations", ref WorkspaceBuffer.AlgBuffer.k);
                ImGui.InputFloat("Step size", ref WorkspaceBuffer.AlgBuffer.d);

                ImGui.End();
            }

            // options & info window
            if (ImGui.Begin("Options & Info",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.75 * Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                if (ImGui.Button("Execute"))
                {
                    Dispatcher.RunThreads();
                }
                if (ImGui.IsItemHovered())
                {
                    ImGui.SameLine();
                    ImGui.TextWrapped("Runs path searching process");
                }

                if (ImGui.Button("Update"))
                {
                    // make pop-up to prevent undesired changes
                    ImGui.OpenPopup("Update?");
                }
                if (ImGui.IsItemHovered())
                {
                    ImGui.SameLine();
                    ImGui.TextWrapped("Updates the entire workspace");
                }

                bool dummy = true;
                if (ImGui.BeginPopupModal("Update?", ref dummy, ImGuiWindowFlags.NoResize))  // TODO: move to separate method
                {
                    ImGui.Text("Do you really want to update the workspace?\nThis will reset the current process.");
                    ImGui.Spacing();
                    ImGui.SetCursorPos(new System.Numerics.Vector2(ImGui.GetWindowSize().X / 2 - 104, ImGui.GetCursorPosY()));
                    if (ImGui.Button("OK", new System.Numerics.Vector2(100, 0)))
                    {
                        // updating workspace and resetting threads
                        Dispatcher.AbortThreads();
                        UpdateWorkspace();
                        Dispatcher.UpdateThreads();

                        ImGui.CloseCurrentPopup();
                    }
                    ImGui.SameLine();
                    if (ImGui.Button("Cancel", new System.Numerics.Vector2(100, 0)))
                    {
                        ImGui.CloseCurrentPopup();
                    }

                    ImGui.EndPopup();
                }

                if (ImGui.Button("Screenshot"))
                {
                    // inform the program that window capturing has been queried
                    Capture = true;
                }
                if (ImGui.IsItemHovered())
                {
                    ImGui.SameLine();
                    ImGui.TextWrapped("Takes a picture of the entire window");
                }

                // save path for captured screenshot
                ImGui.InputText("Save path", ref SavePath, 100);
                TextEdited = ImGui.IsItemActive();

                //if (Manager.Obstacles != null && Manager.Obstacles[0] != null)
                //    ImGui.Text($"Center: {Manager.Obstacles[0].Collider.Center}");  // TODO: when scene is updated, obstacle center is not reset! fix

                // application current framerate
                ImGui.SetCursorScreenPos(new System.Numerics.Vector2(8, Height - 8 - ImGui.CalcTextSize("Framerate:").Y));
                ImGui.Text(string.Format("Framerate: {0:F1} FPS", ImGui.GetIO().Framerate));

                ImGui.End();
            }

            // rendering controller and checking for errors
            controller.Render();
            Util.CheckGLError("End of frame");
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            // check to see if the window is focused
            if (!Focused)
            {
                return;
            }

            // update camera state
            _camera.UpdateViewMatrix();
            _camera.UpdateProjectionMatrix();

            PollKeyboard(_camera, Keyboard.GetState(), e);

            PollMouse(_camera, Mouse.GetCursorState());

            if (ManipLoaded)
            {
                float dt;
                if (forward)
                {
                    dt = (float)e.Time;
                    if (time > 1)
                        forward = false;
                }
                else
                {
                    dt = -(float)e.Time;
                    if (time < -1)
                        forward = true;
                }
                time += dt;

                if (!Manager.Manipulators.All(x => x.Controller.State == ControllerState.Finished))
                {
                    Manager.Obstacles[0].Move(dt * System.Numerics.Vector3.UnitX);
                    //Manager.Obstacles[1].Move(dt * new Vector3(-1, 0, -1));
                    //Manager.Obstacles[2].Move(-dt * new Vector3(-1, -1, -1));
                }

                for (int i = 0; i < Manager.Manipulators.Length; i++)
                {
                    Manipulator manip = Manager.Manipulators[i];

                    // goal
                    if (goal[i] == default)
                    {
                        var goalAttr = manip.Attractors[0];  // TODO: refactor this part
                        var data = new List<System.Numerics.Vector3> { goalAttr.Center };
                        data.AddRange(Primitives.Sphere(goalAttr.Radius, goalAttr.Center, 100));
                        goal[i] = new PlainModel(lineShader, Utils.GL_Convert(data.ToArray(), Color4.Yellow));
                    }

                    // obtained path
                    if (manip.Path != null)
                    {
                        // path may change at any time in control thread; GetRange() guarantees thread sync
                        int count = manip.Path.Count;
                        float[] data = Utils.GL_Convert(manip.Path.GetRange(0, count).ToArray(), Color4.Red);
                        if (path[i] == default)
                        {
                            path[i] = new PlainModel(lineShader, data);
                        }
                        else
                        {
                            path[i].Update(data);
                        }
                    }

                    // random tree
                    if (manip.Tree != null)
                    {
                        // add all elements from addition buffer to the hash set
                        if (manip.Tree.AddBuffer.Contains(null))
                        {
                            int a = 2;  // here, Null does not appear
                        }
                        tree[i].UnionWith(manip.Tree.AddBuffer.DequeueAll());
                        if (manip.Tree.AddBuffer.Contains(null))
                        {
                            int a = 2;  // and here Null appears; 
                                        // seems like AddBuffer loses reference while trimming the tree,
                                        // though it's still not clear why; maybe finalization of a node?
                        }

                        // delete all elements contained in deletion buffer from the hash set
                        tree[i].ExceptWith(manip.Tree.DelBuffer.DequeueAll());
                    }
                }
            }

            base.OnUpdateFrame(e);
        }

        public void PollMouse(Camera camera, MouseState mouse)
        {
            // get mouse position in NDC coordinates
            var mouseNDC = MouseToNDC(mouse);

            if (_firstMove) // this bool variable is initially set to true
            {
                _lastPos = new Vector2(mouse.X, mouse.Y);
                _firstMove = false;
            }
            else if (mouseNDC.X > -1 && mouseNDC.X < 1 &&  // Updating camera only if the mouse is inside the respective viewport
                     mouseNDC.Y > -1 && mouseNDC.Y < 1 &&  // with the left button pressed and if no GUI window is active.
                     mouse.LeftButton == ButtonState.Pressed &&
                     !ImGui.IsWindowFocused(ImGuiFocusedFlags.AnyWindow))  
            {
                // Calculate the offset of the mouse position
                var deltaX = mouse.X - _lastPos.X;
                var deltaY = mouse.Y - _lastPos.Y;

                // Apply the camera pitch and yaw (we clamp the pitch in the camera class)
                _camera.Yaw += deltaX * camera.Sensitivity;
                _camera.Pitch -= deltaY * camera.Sensitivity; // reversed since y-coordinates range from bottom to top
            }

            // updating last mouse position
            _lastPos = new Vector2(mouse.X, mouse.Y);

            // poll widget for interaction
            widget.Poll(_camera, mouse, MouseToNDC(mouse));
        }

        public Vector2 MouseToNDC(MouseState mouse)
        {
            // cursor position relative to window
            var cursorWindow = new Point(mouse.X - X, mouse.Y - Y);

            // take into account window borders
            cursorWindow.X -= (int)(0.25 * Width + 8);  // 8 - indent for resizing feature
            cursorWindow.Y -= 38;  // 38 = 2 * 8 + 22, where 8 - resizing, 22 - main titlebar height

            // return cursor position in NDC coordinates
            return new Vector2(
                (cursorWindow.X / (0.75f * Width) - 0.5f) * 2,
                ((float)(Height - cursorWindow.Y) / Height - 0.5f) * 2);
        }

        public void PollKeyboard(Camera camera, KeyboardState keyboard, FrameEventArgs e)
        {
            // exit program if queried
            if (keyboard.IsKeyDown(Key.Escape))
            {
                Exit();
            }

            if (!TextEdited)
            {
                // panning
                if (keyboard.IsKeyDown(Key.W))
                    camera.Position += camera.Up * camera.Speed * (float)e.Time; // Up 
                if (keyboard.IsKeyDown(Key.S))
                    camera.Position -= camera.Up * camera.Speed * (float)e.Time; // Down
                if (keyboard.IsKeyDown(Key.A))
                    camera.Position -= camera.Right * camera.Speed * (float)e.Time; // Left
                if (keyboard.IsKeyDown(Key.D))
                    camera.Position += camera.Right * camera.Speed * (float)e.Time; // Right
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
                _camera.Fov -= e.DeltaPrecise;  // zooming

            base.OnMouseWheel(e);
        }


        protected override void OnResize(EventArgs e)
        {
            // We need to update the aspect ratio once the window has been resized
            _camera.AspectRatio = (float)(0.75 * Width / Height);

            // report to GUI controller about resizing
            controller.WindowResized(Width, Height);

            base.OnResize(e);
        }

        protected override void OnUnload(EventArgs e)
        {
            // TODO: clear all unmanaged resources

            // freeing all the used resources
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindVertexArray(0);
            GL.UseProgram(0);

            GL.DeleteProgram(_shader.Handle);

            base.OnUnload(e);
        }

        protected void UpdateWorkspace()
        {
            // initializing manager
            Manager.Initialize();

            // initializing all displaying entities
            int manip_length = Manager.Manipulators.Length;
            goal = new PlainModel[manip_length];
            path = new PlainModel[manip_length];
            tree = new HashSet<Logic.PathPlanning.Tree.Node>[manip_length];
            for (int i = 0; i < tree.Length; i++)
            {
                tree[i] = new HashSet<Logic.PathPlanning.Tree.Node>();
            }

            Dispatcher.timers = new Stopwatch[manip_length];
            for (int i = 0; i < manip_length; i++)
            {
                Dispatcher.timers[i] = new Stopwatch();
            }
        }

        // some specific methods for better drawing organization
        // TODO: move somewhere else
        public static PlainModel CreateTreeBranch(System.Numerics.Vector3 p1, System.Numerics.Vector3 p2)
        {
            return new PlainModel(lineShader, Utils.GL_Convert(new System.Numerics.Vector3[] { p1, p2 }, Color4.Black));
        }

        protected override void OnKeyPress(KeyPressEventArgs e)
        {
            base.OnKeyPress(e);

            controller.PressChar(e.KeyChar);
        }
    }
}