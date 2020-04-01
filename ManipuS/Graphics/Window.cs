using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Diagnostics;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Input;
using ImGuiNET;
using Logic;
using System.Threading;
using System.Threading.Tasks;

using Vector3 = Logic.Vector3;
using Vector4 = OpenTK.Vector4;
using Matrix4 = OpenTK.Matrix4;

namespace Graphics
{
    /*public class Entity
    {
        public int VAO, VBO, EBO;
        public Shader Shader;

        public Entity(Shader shader, float[] data, uint[] indices = null)
        {
            Shader = shader;

            // generating array/buffer objects
            VAO = GL.GenVertexArray();
            VBO = GL.GenBuffer();

            GL.BindVertexArray(VAO);

            // binding vertex data to buffer
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);

            // binding indices data to buffer, if presented
            if (indices != null)
            {
                EBO = GL.GenBuffer();
                GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
                GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(float), indices, BufferUsageHint.StaticDraw);
            }

            // configuring all the needed attributes
            var PosAttrib = Shader.GetAttribLocation("aPos");
            GL.VertexAttribVector3er(PosAttrib, 3, VertexAttribVector3erType.Float, false, 7 * sizeof(float), 0);
            GL.EnableVertexAttribArray(PosAttrib);

            var ColAttrib = Shader.GetAttribLocation("aColor");
            GL.VertexAttribVector3er(ColAttrib, 4, VertexAttribVector3erType.Float, false, 7 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(ColAttrib);

            GL.BindVertexArray(0);
        }

        public void Display(Matrix4 model, Action draw)
        {
            // displaying entity with the appropriate draw method
            GL.BindVertexArray(VAO);
            Shader.SetMatrix4("model", model, false);
            draw();
        }
    }*/

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
        Entity grid, gridFloor;
        Entity[] obstacles, boundings, lon;
        Entity[] goal, configs, path;
        Entity cloud, traj, attrGood, attrBad;

        HashSet<Logic.PathPlanning.Tree.Node>[] tree;

        // variables for mouse state processing
        private bool _firstMove = true;
        private Vector2 _lastPos;
        
        // indices of current manipulators' configurations
        private int[] ConfigsCount;

        // manipulators' calculating threads
        private Thread[] threads;
        private bool[] ThreadsRunning;

        // misc variables
        private bool Capture = false;
        private static System.IO.DirectoryInfo projDir = System.IO.Directory.GetParent(Environment.CurrentDirectory).Parent;
        private static System.IO.DirectoryInfo solDir = projDir.Parent;

        private static string ProjectDirectory = projDir.FullName;
        private static string SolutionDirectory = solDir.FullName;

        private static string VertexShader = ProjectDirectory + @"\Graphics\Shaders\VertexShader.glsl";
        private static string FragmentShader = ProjectDirectory + @"\Graphics\Shaders\FragmentShader.glsl";
        private static string LineShader = ProjectDirectory + @"\Graphics\Shaders\LineShader.glsl";

        private static string SavePath = SolutionDirectory + @"\Screenshots";
        private static string NanosuitPath = SolutionDirectory + @"\Resources\Models\nanosuit\nanosuit.obj";
        private static string LinkPath = SolutionDirectory + @"\Resources\Models\manipulator\Link.obj";
        private static string JointPath = SolutionDirectory + @"\Resources\Models\manipulator\Joint.obj";
        private Stopwatch[] timers;
        
        private Thread load;
        private float time = 0;
        private bool forward;
        private bool ManipLoaded = false;

        // 3D model
        Model Crytek;
        static Shader lineShader;

        public Window(int width, int height, GraphicsMode gMode, string title) : 
            base(width, height, gMode, title, GameWindowFlags.Default, DisplayDevice.Default, 4, 6, GraphicsContextFlags.ForwardCompatible) { }

        protected override void OnLoad(EventArgs e)
        {
            //var unptr = Assimp.Unmanaged.AssimpLibrary.Instance.ImportFile(JointPath, Assimp.PostProcessSteps.None, Assimp.Unmanaged.AssimpLibrary.Instance.CreatePropertyStore());
            //var manptr = Assimp.Scene.FromUnmanagedScene(unptr);
            //var ptr = Assimp.Unmanaged.AssimpLibrary.Instance.ApplyPostProcessing(unptr, Assimp.PostProcessSteps.TransformUVCoords);

            // defining ImGui controller
            controller = new ImGuiController(Width, Height);
            
            // retrieving shaders' files
            _shader = new Shader(VertexShader, FragmentShader);

            lineShader = new Shader(VertexShader, LineShader);

            // Camera is 6 units back and has the proper aspect ratio
            _camera = new Camera(Vector3.UnitZ * 6, (float)(0.75 * Width / Height));

            // initializing workspace and manipulators' threads
            //UpdateWorkspace();
            //UpdateThreads();

            // workspace grid
            grid = new Entity(lineShader, gridLines);
            gridFloor = new Entity(lineShader, transparencyMask, new uint[] { 1, 0, 3, 1, 2, 3, 1 });

            base.OnLoad(e);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            controller.Update(this, (float)e.Time);

            // drawing main part (workspace)
            ShowCore(e);

            // drawing GUI
            ShowGUI();

            // capture screen after render if queried
            if (Capture)
            {
                Utils.ScreenCapture(this, SavePath);
                Capture = false;
            }

            SwapBuffers();

            // execute all actions, enqueued while loading a model
            int count = Dispatcher.ActionsQueue.Count;
            for (int i = 0; i < count; i++)
            {
                var action = Dispatcher.ActionsQueue.Dequeue();
                action();
            }
        }


        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            if (!Focused) // check to see if the window is focused
            {
                return;
            }

            var input = Keyboard.GetState();

            // exit program
            if (input.IsKeyDown(Key.Escape))
            {
                Exit();
            }

            // camera parameters
            const float cameraSpeed = 3f;
            const float sensitivity = 0.2f;

            // panning
            if (input.IsKeyDown(Key.W))
                _camera.Position += _camera.Up * cameraSpeed * (float)e.Time; // Up 
            if (input.IsKeyDown(Key.S))
                _camera.Position -= _camera.Up * cameraSpeed * (float)e.Time; // Down
            if (input.IsKeyDown(Key.A))
                _camera.Position -= _camera.Right * cameraSpeed * (float)e.Time; // Left
            if (input.IsKeyDown(Key.D))
                _camera.Position += _camera.Right * cameraSpeed * (float)e.Time; // Right

            // Get the mouse state
            var mouse = Mouse.GetState();

            var cursor = Mouse.GetCursorState();
            var window = Location;
            var CursorWindow = new System.Drawing.Point(cursor.X - window.X, cursor.Y - window.Y);  // cursor position relative to window
            var GuiActive = ImGui.IsWindowFocused(ImGuiFocusedFlags.AnyWindow);

            if (_firstMove) // this bool variable is initially set to true
            {
                _lastPos = new Vector2(mouse.X, mouse.Y);
                _firstMove = false;
            }
            else if (CursorWindow.X > (int)(0.25 * Width) + 8 &&  // Updating camera only if the mouse is inside the respective viewport
                     CursorWindow.X < Width + 8 &&                // with the left button pressed and if no GUI window is active.
                     CursorWindow.Y > 31 &&                       // Of indents: for (1000, 600) client size we have (1016, 639) actual size, where:
                     CursorWindow.Y < Height + 31 &&              // 8 - indent for resizing feature, 39 - 2 * 8 = 23 - main titlebar height
                     mouse.LeftButton == ButtonState.Pressed && !GuiActive)
            {
                // Calculate the offset of the mouse position
                var deltaX = mouse.X - _lastPos.X;
                var deltaY = mouse.Y - _lastPos.Y;

                // Apply the camera pitch and yaw (we clamp the pitch in the camera class)
                _camera.Yaw += deltaX * sensitivity;
                _camera.Pitch -= deltaY * sensitivity; // reversed since y-coordinates range from bottom to top
            }

            // updating last mouse position
            _lastPos = new Vector2(mouse.X, mouse.Y);

            base.OnUpdateFrame(e);
        }
        
        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            base.OnMouseMove(e);
        }
        
        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            // applying zoom only if no GUI window is currently hovered over
            if (!ImGui.IsWindowHovered(ImGuiHoveredFlags.AnyWindow))
                _camera.Fov -= e.DeltaPrecise;  // zooming

            base.OnMouseWheel(e);
        }


        protected override void OnResize(EventArgs e)
        {
            // We need to update the aspect ratio once the window has been resized
            _camera.AspectRatio = (float)(0.75 * Width / Height);

            base.OnResize(e);

            // reporting GUI controller about resizing
            controller.WindowResized(Width, Height);
        }

        protected override void OnUnload(EventArgs e)
        {
            // freeing all the used resources
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindVertexArray(0);
            GL.UseProgram(0);

            GL.DeleteProgram(_shader.Handle);

            base.OnUnload(e);
        }

        protected void ShowCore(FrameEventArgs e)
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
            _shader.SetMatrix4("view", _camera.GetViewMatrix(), false);
            _shader.SetMatrix4("projection", _camera.GetProjectionMatrix(), false);

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
            lineShader.SetMatrix4("view", _camera.GetViewMatrix(), false);
            lineShader.SetMatrix4("projection", _camera.GetProjectionMatrix(), false);
            lineShader.SetVector3("color", Vector3.One);

            Matrix4 model;

            if (ManipLoaded)
            {
                // obstacles & colliders
                if (obstacles.Contains(null))
                {
                    for (int i = 0; i < obstacles.Length; i++)
                    {
                        obstacles[i] = new Entity(lineShader, GL_Convert(Manager.Obstacles[i].Data, Vector4.One));

                        switch (Manager.Obstacles[i].Collider.Shape)
                        {
                            case ColliderShape.Box:
                                boundings[i] = new Entity(lineShader, GL_Convert(Manager.Obstacles[i].Collider.Data, new Vector4(Vector3.UnitY, 1.0f)), new uint[]
                                {
                                    0, 1, 2, 3, 0, 4, 5, 1, 5, 6, 2, 6, 7, 3, 7, 4
                                });
                                break;
                            case ColliderShape.Sphere:
                                boundings[i] = new Entity(lineShader, GL_Convert(Manager.Obstacles[i].Collider.Data, new Vector4(Vector3.UnitY, 1.0f)));
                                lon[i] = new Entity(lineShader, GL_Convert(Manager.Obstacles[i].Collider.Data, new Vector4(Vector3.UnitY, 1.0f)), ((Sphere)Manager.Obstacles[i].Collider).indicesLongitude);
                                break;
                        }
                    }
                }
                else
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
                    
                    model = Matrix4.CreateTranslation(new Vector3(time, 0, 0));  // TODO: obstacle movement should be done in a separate method, with high frequency
                    for (int i = 0; i < obstacles.Length; i++)
                    {
                        Manager.Obstacles[i].Move(Vector3.UnitX, dt);

                        obstacles[i].Display(model, () =>
                        {
                            GL.DrawArrays(PrimitiveType.Points, 0, Manager.Obstacles[i].Data.Length);
                        });

                        if (Dispatcher.WorkspaceBuffer.ObstBuffer[i].ShowBounding)
                        {
                            boundings[i].Display(model, Manager.Obstacles[i].Collider.Draw);
                            lon[i].Display(model, ((Sphere)Manager.Obstacles[i].Collider).DrawLongitudes);
                        }
                    }
                }

                for (int j = 0; j < Manager.Manipulators.Length; j++)
                {
                    Manipulator manip = Manager.Manipulators[j];

                    // goal
                    if (goal[j] == null)
                    {
                        if (manip.States["Goal"])
                        {
                            List<Vector3> MainAttr = new List<Vector3> { manip.GoodAttractors[0].Center };
                            MainAttr.AddRange(manip.GoodAttractors[0].Area);
                            goal[j] = new Entity(lineShader, GL_Convert(MainAttr.ToArray(), new Vector4(1.0f, 1.0f, 0.0f, 1.0f)));
                        }
                    }
                    else
                    {
                        model = Matrix4.Identity;
                        goal[j].Display(model, () =>
                        {
                            GL.PointSize(5);
                            GL.DrawArrays(PrimitiveType.Points, 0, 1);
                            GL.PointSize(1);
                            GL.DrawArrays(PrimitiveType.Points, 1, manip.GoodAttractors[0].Area.Length);
                        });
                    }

                    // path
                    if (manip.States["Path"] && manip.Path != null)
                    {
                        // path may change at any time in control thread; GetRange() guarantees thread sync
                        int count = manip.Path.Count;
                        path[j] = new Entity(lineShader, GL_Convert(manip.Path.GetRange(0, count).ToArray(), new Vector4(Vector3.UnitX, 1.0f)));

                        model = Matrix4.Identity;
                        path[j].Display(model, () =>
                        {
                            GL.DrawArrays(PrimitiveType.LineStrip, 0, count);
                        });
                    }

                    // random tree
                    if (manip.Tree != null)
                    {
                        // block the manager thread while updating tree
                        Dispatcher.UpdateTree.Reset();

                        // add all elements from addition buffer to the hash set
                        var add = new List<Logic.PathPlanning.Tree.Node>(manip.Tree.AddBuffer);
                        tree[j].UnionWith(add);
                        manip.Tree.AddBuffer.Clear();

                        // delete all elements contained in deletion buffer from the hash set
                        var del = new List<Logic.PathPlanning.Tree.Node>(manip.Tree.DelBuffer);
                        tree[j].ExceptWith(del);
                        manip.Tree.DelBuffer.Clear();

                        // unblock the manager thread to continue calculations
                        Dispatcher.UpdateTree.Set();
                    }

                    if (Dispatcher.WorkspaceBuffer.JointBuffer[j].ShowTree)
                    {
                        model = Matrix4.Identity;
                        foreach (var node in tree[j])
                        {
                            if (node.Entity == null)
                                node.Entity = CreateTreeBranch(node.p, node.Parent.p);

                            node.Entity.Display(model, () =>
                            {
                                GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                            });
                        }
                    }

                    // draw manipulator configuration
                    manip.Draw(_shader);
                }
            }

            // workspace grid
            model = Matrix4.Identity;
            grid.Display(model, () =>
            {
                GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                GL.DrawArrays(PrimitiveType.LineStrip, 2, 2);
            });
            for (int i = 1; i < 11; i++)
            {
                model = Matrix4.CreateTranslation(Vector3.UnitZ * i);
                grid.Display(model, () => { GL.DrawArrays(PrimitiveType.LineStrip, 0, 2); });
                model = Matrix4.CreateTranslation(Vector3.UnitZ * -i);
                grid.Display(model, () => { GL.DrawArrays(PrimitiveType.LineStrip, 0, 2); });

                model = Matrix4.CreateTranslation(Vector3.UnitX * i);
                grid.Display(model, () => { GL.DrawArrays(PrimitiveType.LineStrip, 2, 2); });
                model = Matrix4.CreateTranslation(Vector3.UnitX * -i);
                grid.Display(model, () => { GL.DrawArrays(PrimitiveType.LineStrip, 2, 2); });
            }

            model = Matrix4.Identity;
            gridFloor.Display(model, () =>
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

        protected void ShowGUI()
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
                            /*load = new Thread(() => 
                            {
                                for (int i = 0; i < Dispatcher.WorkspaceBuffer.LinkBuffer.Length; i++)
                                {
                                    Dispatcher.WorkspaceBuffer.JointBuffer[i].Model = new Model(JointPath);
                                    Dispatcher.WorkspaceBuffer.LinkBuffer[i].Model = new Model(LinkPath);
                                }

                                // wait for loading process to finish
                                while (Dispatcher.ActionsQueue.Count != 0) { }  // TODO: remove redundant checks for ActionsQueue in this file (because ManipLoaded == true only when ActionsQueue.Count == 0)
                                
                                // update workspace with newly loaded model
                                UpdateWorkspace();
                                UpdateThreads();
                                ManipLoaded = true;
                                //Crytek = new Model(ManipPath);
                            })
                            {
                                Name = "LoadModel",
                                IsBackground = true
                            };
                            load.Start();*/
                            Dispatcher.ActiveTasks.Add(Task.Run(() =>
                            {
                                for (int i = 0; i < Dispatcher.WorkspaceBuffer.LinkBuffer.Length; i++)
                                {
                                    Dispatcher.WorkspaceBuffer.JointBuffer[i].Model = new Model(JointPath);
                                    Dispatcher.WorkspaceBuffer.LinkBuffer[i].Model = new Model(LinkPath);
                                }

                                // wait for loading process to finish
                                while (Dispatcher.ActionsQueue.Count != 0) { }  // TODO: remove redundant checks for ActionsQueue in this file (because ManipLoaded == true only when ActionsQueue.Count == 0)
                                // TODO: better to put WaitHandle here instead of emptiness check

                                // update workspace with newly loaded model
                                UpdateWorkspace();
                                UpdateThreads();
                                ManipLoaded = true;
                                //Crytek = new Model(ManipPath);
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
            /*if (ImGui.Begin("Manipulators",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize | 
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, 0));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                var MD = Manager.MD;

                for (int j = 0; j < Manager.Manipulators.Length; j++)
                {
                    if (ImGui.TreeNode($"Manip {j}"))
                    {
                        ImGui.Text($"Time spent: {(float)timers[j].ElapsedMilliseconds / 1000} s");

                        int count = Manager.Manipulators[j].Tree == null ? 0 : Manager.Manipulators[j].Tree.Count;
                        ImGui.Checkbox($"Show tree ({count} verts)", ref MD[j].ShowTree);
                        ImGui.InputFloat3("Goal", ref MD[j].Goal);
                        ImGui.InputFloat3("Base", ref MD[j].Base);
                        ImGui.InputInt("Links number", ref MD[j].N);

                        if (ImGui.TreeNode("Links"))
                        {
                            for (int i = 0; i < MD[j].N; i++)
                            {
                                ImGui.InputFloat($"Link {i}", ref MD[j].l[i]);
                            }
                            ImGui.TreePop();
                        }

                        if (ImGui.TreeNode("Initial GC (deg)"))
                        {
                            for (int i = 0; i < MD[j].N; i++)
                            {
                                ImGui.InputFloat($"GC {i}", ref MD[j].q[i]);
                            }
                            ImGui.TreePop();
                        }

                        if (ImGui.TreeNode("GC ranges (deg)"))
                        {
                            for (int i = 0; i < MD[j].N; i++)
                            {
                                ImGui.InputFloat2($"GC {i}", ref MD[j].q_ranges[i]);
                            }
                            ImGui.TreePop();
                        }

                        if (ImGui.TreeNode("D-H params"))
                        {
                            for (int i = 0; i < MD[j].N; i++)
                            {
                                ImGui.InputFloat4($"D-H {i}", ref MD[j].DH[i]);
                            }
                            ImGui.TreePop();
                        }

                        ImGui.TreePop();
                    }
                }

                ImGui.End();
            }*/

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
                            ImGui.Checkbox("Show collider", ref Dispatcher.WorkspaceBuffer.ObstBuffer[i].ShowBounding);
                            ImGui.InputFloat("Radius", ref Dispatcher.WorkspaceBuffer.ObstBuffer[i].r);
                            ImGui.InputFloat3("Center", ref Dispatcher.WorkspaceBuffer.ObstBuffer[i].c);
                            ImGui.InputInt("Vector3s number", ref Dispatcher.WorkspaceBuffer.ObstBuffer[i].Vector3s_num);

                            ImGui.TreePop();
                        }
                    }
                }
                else
                {
                    ImGui.Text("No obstacles in the scene.");
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

                ImGui.InputInt("Attractors number", ref Dispatcher.WorkspaceBuffer.AlgBuffer.AttrNum);
                ImGui.NewLine();
                ImGui.Text("Inverse kinematics solver:");
                ImGui.PushID(0);
                ImGui.InputInt("Iterations", ref Dispatcher.WorkspaceBuffer.AlgBuffer.MaxTime);
                ImGui.InputFloat("Precision", ref Dispatcher.WorkspaceBuffer.AlgBuffer.Precision);
                ImGui.InputFloat("Step size (deg)", ref Dispatcher.WorkspaceBuffer.AlgBuffer.StepSize);
                ImGui.NewLine();
                ImGui.PushID(1);
                ImGui.Text("RRT path planner:");
                ImGui.InputInt("Iterations", ref Dispatcher.WorkspaceBuffer.AlgBuffer.k);
                ImGui.InputFloat("Step size", ref Dispatcher.WorkspaceBuffer.AlgBuffer.d);

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
                    RunThreads();
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
                if (ImGui.BeginPopupModal("Update?", ref dummy, ImGuiWindowFlags.NoResize))
                {                    
                    ImGui.Text("Do you really want to update the workspace?\nThis will reset the current process.");
                    ImGui.Spacing();
                    ImGui.SetCursorPos(new System.Numerics.Vector2(ImGui.GetWindowSize().X / 2 - 104, ImGui.GetCursorPosY()));
                    if (ImGui.Button("OK", new System.Numerics.Vector2(100, 0)))
                    {
                        // updating workspace and resetting threads
                        AbortThreads();
                        UpdateWorkspace();
                        UpdateThreads();

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

                // application current framerate
                ImGui.SetCursorScreenPos(new System.Numerics.Vector2(8, Height - 8 - ImGui.CalcTextSize("Framerate:").Y));
                ImGui.Text(string.Format("Framerate: {0:F1} FPS", ImGui.GetIO().Framerate));

                ImGui.End();
            }

            // rendering controller and checking for errors
            controller.Render();
            Util.CheckGLError("End of frame");
        }

        protected void UpdateWorkspace()
        {
            // initializing manager
            Manager.Initialize();

            // initializing all displaying entities
            int obst_length = Manager.Obstacles.Length;
            obstacles = new Entity[obst_length];
            boundings = new Entity[obst_length];
            lon = new Entity[obst_length];

            int manip_length = Manager.Manipulators.Length;
            goal = new Entity[manip_length];
            configs = new Entity[manip_length];
            path = new Entity[manip_length];
            //tree = new List<Entity>[manip_length];
            tree = new HashSet<Logic.PathPlanning.Tree.Node>[manip_length];
            for (int i = 0; i < tree.Length; i++)
            {
                tree[i] = new HashSet<Logic.PathPlanning.Tree.Node>();  //new List<Entity>();
            }

            ConfigsCount = new int[manip_length];

            timers = new Stopwatch[manip_length];
            for (int i = 0; i < manip_length; i++)
            {
                timers[i] = new Stopwatch();
            }
        }

        protected void UpdateThreads()  // TODO: for WaitHandles Tasks would be better than Threads
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
                        Manager.Execute(Manager.Manipulators[index]);
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

        protected void RunThreads()
        {
            // running all threads
            for (int i = 0; i < threads.Length; i++)
            {
                threads[i].Start();
            }
        }

        protected void AbortThreads()
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

        protected void ScreenCapture()
        {
            // taking a picture of a viewport
            byte[,,] img = new byte[Height, Width, 3];
            GL.ReadPixels(0, 0, Width, Height, PixelFormat.Rgb, PixelType.UnsignedByte, img);

            System.Drawing.Bitmap bitmap = new System.Drawing.Bitmap(Width, Height);
            for (int i = 0; i < Height; i++)
            {
                for (int j = 0; j < Width; j++)
                {
                    bitmap.SetPixel(j, Height - 1 - i, System.Drawing.Color.FromArgb(img[i, j, 0], img[i, j, 1], img[i, j, 2]));
                }
            }

            // saving captured image
            bitmap.Save(SavePath + ".bmp", System.Drawing.Imaging.ImageFormat.Bmp);

            Capture = false;
        }

        static protected float[] GL_Convert(Vector3[] data, Vector4 color)
        {
            // converting program data to OpenGL buffer format
            float[] res = new float[data.Length * 7];

            for (int i = 0; i < data.Length; i++)
            {
                res[7 * i] = data[i].X;
                res[7 * i + 1] = data[i].Y;
                res[7 * i + 2] = data[i].Z;
                res[7 * i + 3] = color.X;
                res[7 * i + 4] = color.Y;
                res[7 * i + 5] = color.Z;
                res[7 * i + 6] = color.W;
            }

            return res;
        }

        // some specific methods for better drawing organization
        public static Entity CreateTreeBranch(Vector3 p1, Vector3 p2)
        {
            return new Entity(lineShader, GL_Convert(new Vector3[] { p1, p2 }, new Vector4(Vector3.Zero, 1.0f)));
        }

        protected override void OnKeyPress(KeyPressEventArgs e)
        {
            base.OnKeyPress(e);

            controller.PressChar(e.KeyChar);
        }
    }
}