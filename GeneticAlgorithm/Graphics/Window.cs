using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Input;
using ImGuiNET;
using Logic;

namespace Graphics
{
    public class Window : GameWindow
    {
        public class Entity
        {
            public int VBO, EBO, VAO;

            public Entity(float[] data, uint[] indices = null)
            {
                //generating array/buffer objects
                VBO = GL.GenBuffer();
                VAO = GL.GenVertexArray();

                GL.BindVertexArray(VAO);

                //binding vertex data to buffer
                GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
                GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);

                //binding indices data to buffer, if presented
                if (indices != null)
                {
                    EBO = GL.GenBuffer();
                    GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
                    GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(float), indices, BufferUsageHint.StaticDraw);
                }

                //configuring all the needed attributes
                var PosAttrib = _shader.GetAttribLocation("aPos");
                GL.VertexAttribPointer(PosAttrib, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
                GL.EnableVertexAttribArray(PosAttrib);

                var ColAttrib = _shader.GetAttribLocation("aColor");
                GL.VertexAttribPointer(ColAttrib, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
                GL.EnableVertexAttribArray(ColAttrib);

                GL.BindVertexArray(0);
            }

            public void Display(Matrix4 model, Action draw)
            {
                GL.BindVertexArray(VAO);
                _shader.SetMatrix4("model", model);
                draw();
            }
        }

        ImGuiController controller;

        private float[] gridX_lines =
        {
            10.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
            -10.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        };

        private float[] gridY_lines =
        {
            0.0f, 0.0f, 10.0f, 1.0f, 1.0f, 1.0f,
            0.0f, 0.0f, -10.0f, 1.0f, 1.0f, 1.0f,
        };

        Entity gridX, gridY;
        Entity[] obstacles, boundings;
        Entity[] goal, joints, path;
        Entity[][] attractors;
        List<Entity>[] tree;

        private static Shader _shader;
        
        private Camera _camera;
        private bool _firstMove = true;
        private Vector2 _lastPos;

        private double _time;
        
        private int[] JointsCount;
        private bool ShowAttractors, num1_prev, f_prev;

        private Thread[] threads;
        private Attractor[][] AttractorsLoc;

        private bool demo = false, Capture = false;
        
        public Window(int width, int height, string title, GraphicsMode gMode) : base(width, height, gMode, title,
                                    GameWindowFlags.Default,
                                    DisplayDevice.Default,
                                    4, 6, GraphicsContextFlags.ForwardCompatible) { }

        protected override void OnLoad(EventArgs e)
        {
            // define ImGui controller
            controller = new ImGuiController(Width, Height);

            GL.Enable(EnableCap.DepthTest);

            _shader = new Shader(@"C:\Users\Dan\source\repos\R3T\Shaders\VertexShader.txt",
                @"C:\Users\Dan\source\repos\R3T\Shaders\FragmentShader.txt");
            _shader.Use();

            // We initialize the camera so that it is 3 units back from where the rectangle is
            // and give it the proper aspect ratio
            _camera = new Camera(Vector3.UnitZ * 6, (float)(0.75 * Width / Height));

            // We make the mouse cursor invisible so we can have proper FPS-camera movement
            //CursorVisible = false;

            // initializing workspace and manipulators' threads
            UpdateWorkspace();
            UpdateThreads();

            // coordinate frame grid
            gridX = new Entity(gridX_lines);
            gridY = new Entity(gridY_lines);

            var input = Keyboard.GetState();
            num1_prev = input.IsKeyDown(Key.Number1);
            f_prev = input.IsKeyDown(Key.F);

            base.OnLoad(e);
        }


        protected override void OnRenderFrame(FrameEventArgs e)
        {
            controller.Update(this, (float)e.Time);

            _time += 4.0 * e.Time;

            ShowCore(e);

            ShowGUI();

            if (Capture)
                ScreenCapture();

            SwapBuffers();
        }


        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            if (!Focused) // check to see if the window is focused
            {
                return;
            }

            var input = Keyboard.GetState();

            if (input.IsKeyDown(Key.Escape))
            {
                Exit();
            }

            const float cameraSpeed = 3f;
            const float sensitivity = 0.2f;

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

            if (_firstMove) // this bool variable is initially set to true
            {
                _lastPos = new Vector2(mouse.X, mouse.Y);
                _firstMove = false;
            }
            else if (CursorWindow.X > (int)(0.25 * Width) + 8 && 
                     CursorWindow.X < Width + 8 &&
                     CursorWindow.Y > 31 && 
                     CursorWindow.Y < Height + 31 && 
                     mouse.LeftButton == ButtonState.Pressed && !ImGui.IsWindowFocused(ImGuiFocusedFlags.AnyWindow))  // for (1000, 600) client size we have (1016, 639) actual size, where: 8 - indent for resizing feature, 39 - 2 * 8 = 23 - main titlebar height
            {
                // Calculate the offset of the mouse position
                var deltaX = mouse.X - _lastPos.X;
                var deltaY = mouse.Y - _lastPos.Y;

                // Apply the camera pitch and yaw (we clamp the pitch in the camera class)
                _camera.Yaw += deltaX * sensitivity;
                _camera.Pitch -= deltaY * sensitivity; // reversed since y-coordinates range from bottom to top
            }

            _lastPos = new Vector2(mouse.X, mouse.Y);

            base.OnUpdateFrame(e);
        }
        
        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            if (Focused) // check to see if the window is focused
            {
                //Mouse.SetPosition(X + Width / 2f, Y + Height / 2f);  // set mouse position back to center
            }

            base.OnMouseMove(e);
        }
        
        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            if (!ImGui.IsWindowHovered(ImGuiHoveredFlags.AnyWindow))
                _camera.Fov -= e.DeltaPrecise;  // zooming
            base.OnMouseWheel(e);
        }


        protected override void OnResize(EventArgs e)
        {
            // We need to update the aspect ratio once the window has been resized
            _camera.AspectRatio = (float)(0.75 * Width / Height);
            base.OnResize(e);

            controller.WindowResized(Width, Height);
        }

        protected override void OnUnload(EventArgs e)
        {
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindVertexArray(0);
            GL.UseProgram(0);

            GL.DeleteProgram(_shader.Handle);

            base.OnUnload(e);
        }

        protected void ShowCore(FrameEventArgs e)
        {
            GL.Viewport((int)(0.25 * Width), 0, (int)(0.75 * Width), Height);

            GL.Enable(EnableCap.ScissorTest);
            GL.Scissor((int)(0.25 * Width), 0, (int)(0.75 * Width), Height);
            GL.ClearColor(0.2f, 0.3f, 0.3f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            GL.Disable(EnableCap.ScissorTest);

            _shader.Use();
            _shader.SetBool("use_color", 1);

            _shader.SetMatrix4("view", _camera.GetViewMatrix());
            _shader.SetMatrix4("projection", _camera.GetProjectionMatrix());

            Matrix4 model;

            // X axis grid lines
            model = Matrix4.Identity;
            gridX.Display(model, () =>
            {
                GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
            });
            for (int i = 1; i < 11; i++)
            {
                model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitZ * i);
                gridX.Display(model, () =>
                {
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                });

                model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitZ * -i);
                gridX.Display(model, () =>
                {
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                });
            }

            // Y axis grid lines
            model = Matrix4.Identity;
            gridY.Display(model, () =>
            {
                GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
            });
            for (int i = 1; i < 11; i++)
            {
                model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitX * i);
                gridY.Display(model, () =>
                {
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                });

                model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitX * -i);
                gridY.Display(model, () =>
                {
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                });
            }

            // obstacles + boundings
            if (obstacles == null)
            {
                obstacles = new Entity[Manager.Obstacles.Length];
                boundings = new Entity[Manager.Obstacles.Length];
                for (int i = 0; i < obstacles.Length; i++)
                {
                    obstacles[i] = new Entity(GL_Convert(Manager.Obstacles[i].Data, Vector3.One));
                    boundings[i] = new Entity(GL_Convert(Manager.Obstacles[i].Bounding, Vector3.UnitY), new uint[]
                        {
                                0, 1, 2, 3, 0, 4, 5, 1, 5, 6, 2, 6, 7, 3, 7, 4
                        });
                }
            }
            else
            {
                model = Matrix4.Identity;
                for (int i = 0; i < obstacles.Length; i++)
                {
                    obstacles[i].Display(model, () =>
                    {
                        GL.DrawArrays(PrimitiveType.Points, 0, Manager.Obstacles[i].Data.Length);
                    });
                }

                for (int i = 0; i < boundings.Length; i++)
                {
                    boundings[i].Display(model, () =>
                    {
                        GL.DrawElements(BeginMode.LineStrip, 16, DrawElementsType.UnsignedInt, 0);
                    });
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
                        List<Point> MainAttr = new List<Point> { manip.Attractors[0].Center };
                        MainAttr.AddRange(manip.Attractors[0].Area);
                        goal[j] = new Entity(GL_Convert(MainAttr.ToArray(), new Vector3(1, 1, 0)));
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
                        GL.DrawArrays(PrimitiveType.Points, 1, manip.Attractors[0].Area.Length);
                    });
                }

                // attractors
                /*if (attractors[j] == null)
                {
                    if (manip.States["Attractors"])
                    {
                        attractors[j] = new Entity[manip.Attractors.Count - 1];
                        for (int i = 1; i < manip.Attractors.Count; i++)
                        {
                            var point = GL_Convert(new Point[] { manip.Attractors[i].Center }, Vector3.UnitX);
                            var area = GL_Convert(manip.Attractors[i].Area, new Vector3(1, 0, 1));
                            attractors[j][i - 1] = new Entity(point.Concat(area).ToArray());
                        }

                        AttractorsLoc[j] = manip.Attractors.GetRange(1, manip.Attractors.Count - 1).ToArray();
                    }
                }
                else if (ShowAttractors)
                {
                    model = Matrix4.Identity;
                    for (int i = 0; i < attractors[j].Length; i++)
                    {
                        attractors[j][i].Display(model, () =>
                        {
                            GL.PointSize(3);
                            GL.DrawArrays(PrimitiveType.Points, 0, 1);
                            GL.PointSize(1);
                            GL.DrawArrays(PrimitiveType.Points, 1, AttractorsLoc[j][i].Area.Length);
                        });
                    }
                }*/

                //random tree
                if (manip.Tree != null)
                {
                    if (manip.Tree.Buffer.Count != 0)
                    {
                        for (int i = 0; i < manip.Tree.Buffer.Count; i++)
                        {
                            tree[j].Add(new Entity(GL_Convert(new Point[] { manip.Tree.Buffer[i].p, manip.Tree.Buffer[i].Parent.p }, Vector3.Zero)));
                        }
                        manip.Tree.Buffer.Clear();
                    }
                }

                if (Manager.MD[j].ShowTree)
                {
                    model = Matrix4.Identity;
                    for (int i = 0; i < tree[j].Count; i++)
                    {
                        tree[j][i].Display(model, () =>
                        {
                            GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                        });
                    }
                }

                // checking if the thread has aborted
                if (!threads[j].IsAlive)
                {
                    // path
                    if (path[j] == null)
                    {
                        if (manip.States["Path"])
                            path[j] = new Entity(GL_Convert(manip.Path.ToArray(), Vector3.UnitX));
                    }
                    else
                    {
                        model = Matrix4.Identity;
                        path[j].Display(model, () =>
                        {
                            GL.DrawArrays(PrimitiveType.LineStrip, 0, manip.Path.Count);
                        });
                    }
                }

                // joints
                if (manip.Joints.Count != 0)
                    joints[j] = new Entity(GL_Convert(manip.Joints[JointsCount[j] < manip.Joints.Count - 1 ? JointsCount[j]++ : JointsCount[j]], new Vector3(1.0f, 0.5f, 0.0f)));

                model = Matrix4.Identity;
                joints[j].Display(model, () =>
                {
                    GL.PointSize(5);
                    GL.DrawArrays(PrimitiveType.Points, 0, 1);
                    GL.PointSize(1);
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, manip.DH.Length + 1);
                });
            }

            if (demo)
                ImGui.ShowDemoWindow();

            base.OnRenderFrame(e);
        }

        protected void ShowGUI()
        {
            GL.Viewport(0, 0, Width, Height);

            GL.Enable(EnableCap.ScissorTest);
            GL.Scissor(0, 0, (int)(0.25 * Width), Height);
            GL.ClearColor(0.3f, 0.3f, 0.3f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            GL.Disable(EnableCap.ScissorTest);

            /*if (ImGui.BeginMainMenuBar())
            {
                if (ImGui.BeginMenu("File"))
                {
                    ImGui.MenuItem("Open...");
                    ImGui.MenuItem("Save");
                    ImGui.MenuItem("Save as...");
                    ImGui.EndMenu();
                }
                ImGui.EndMainMenuBar();
            }*/

            if (ImGui.Begin("Manipulators",
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
                        ImGui.Checkbox("Show tree", ref MD[j].ShowTree);
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

                        if (ImGui.TreeNode("Initial GC (degrees)"))
                        {
                            for (int i = 0; i < MD[j].N; i++)
                            {
                                ImGui.InputFloat($"GC {i}", ref MD[j].q[i]);
                            }
                            ImGui.TreePop();
                        }

                        if (ImGui.TreeNode("GC ranges"))
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
            }

            if (ImGui.Begin("Obstacles",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize |
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.25 * Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                var OD = Manager.OD;

                for (int i = 0; i < Manager.Obstacles.Length; i++)
                {
                    if (ImGui.TreeNode($"Obst {i}"))
                    {
                        ImGui.Checkbox("Show bounding", ref OD[i].ShowBounding);
                        ImGui.InputFloat("Radius", ref OD[i].r);
                        ImGui.InputFloat3("Center", ref OD[i].c);
                        ImGui.InputInt("Points number", ref OD[i].points_num);

                        ImGui.TreePop();
                    }
                }

                ImGui.End();
            }

            if (ImGui.Begin("Algorithm",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize | 
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.5 * Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                ImGui.InputInt("Attractors number", ref Manager.AD.AttrNum);
                ImGui.NewLine();
                ImGui.Text("Inverse kinematics solver:");
                ImGui.PushID(0);
                ImGui.InputInt("Iterations", ref Manager.AD.MaxTime);
                ImGui.InputFloat("Precision", ref Manager.AD.Precision);
                ImGui.InputFloat("Step size", ref Manager.AD.StepSize);
                ImGui.NewLine();
                ImGui.PushID(1);
                ImGui.Text("RRT path planner:");
                ImGui.InputInt("Iterations", ref Manager.AD.k);
                ImGui.InputFloat("Step size", ref Manager.AD.d);

                ImGui.End();
            }

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

                if (ImGui.BeginPopupModal("Update?"))
                {
                    ImGui.Text("Do you really want to update the workspace? This will reset the current process.");
                    if (ImGui.Button("OK"))
                    {
                        // update workspace and reset threads
                        AbortThreads();
                        UpdateWorkspace();
                        UpdateThreads();

                        ImGui.CloseCurrentPopup();
                    }
                    ImGui.SameLine();
                    if (ImGui.Button("Cancel"))
                    {
                        ImGui.CloseCurrentPopup();
                    }

                    ImGui.EndPopup();
                }

                if (ImGui.Button("Screenshot"))
                {
                    Capture = true;
                }
                if (ImGui.IsItemHovered())
                {
                    ImGui.SameLine();
                    ImGui.TextWrapped("Takes a picture of the entire window");
                }

                ImGui.Checkbox("Demo", ref demo);

                // application current framerate
                ImGui.SetCursorScreenPos(new System.Numerics.Vector2(8, Height - 8 - ImGui.CalcTextSize("Framerate:").Y));
                ImGui.Text(string.Format("Framerate: {0:F1} FPS", ImGui.GetIO().Framerate));

                ImGui.End();
            }

            controller.Render();
            Util.CheckGLError("End of frame");
        }

        protected void UpdateWorkspace()
        {
            // initializing manager
            Manager.Initialize();

            // initializing all displaying entities
            int length = Manager.Manipulators.Length;
            goal = new Entity[length];
            joints = new Entity[length];
            path = new Entity[length];
            attractors = new Entity[length][];
            tree = new List<Entity>[length];
            for (int i = 0; i < tree.Length; i++)
            {
                tree[i] = new List<Entity>();
            }

            AttractorsLoc = new Attractor[length][];

            JointsCount = new int[length];
        }

        protected void UpdateThreads()
        {
            // enabling/disabling specific threads for calculating paths for manipulators
            threads = new Thread[Manager.Manipulators.Length];
            for (int i = 0; i < threads.Length; i++)
            {
                int index = i;
                threads[i] = new Thread(() => Manager.Execute(Manager.Manipulators[index]))
                {
                    Name = $"Manipulator {index}",
                    IsBackground = true
                };
            }
        }

        protected void RunThreads()
        {
            // run all threads
            foreach (var thread in threads)
                thread.Start();
        }

        protected void AbortThreads()
        {
            // abort all threads if presented
            if (threads != null)
            {
                foreach (var thread in threads)
                thread.Abort();

                while (threads.Any((t) => { return t.ThreadState != ThreadState.Aborted; })) { }
            }
        }

        protected void ScreenCapture()
        {
            // take a picture of a viewport and save it
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

            bitmap.Save("D://RRT_Snap.bmp", System.Drawing.Imaging.ImageFormat.Bmp);

            Capture = false;
        }

        protected float[] GL_Convert(Point[] data, Vector3 color)
        {
            float[] res = new float[data.Length * 6];

            for (int i = 0; i < data.Length; i++)
            {
                res[6 * i] = (float)data[i].x;
                res[6 * i + 1] = (float)data[i].y;
                res[6 * i + 2] = (float)data[i].z;
                res[6 * i + 3] = color.X;
                res[6 * i + 4] = color.Y;
                res[6 * i + 5] = color.Z;
            }

            return res;
        }

        protected override void OnKeyPress(KeyPressEventArgs e)
        {
            base.OnKeyPress(e);

            controller.PressChar(e.KeyChar);
        }
    }
}