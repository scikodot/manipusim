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
using Physics;
using BulletSharp;
using System.Threading;
using MathNet.Numerics;

namespace Graphics
{
    public class MainWindow : GameWindow
    {
        // main graphics objects
        private ImGuiController controller;
        private Camera _camera;

        // workspace grid
        private readonly MeshVertex[] gridLines =
        {
            new MeshVertex { Position = new Vector3(10.0f, 0.0f, 0.0f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-10.0f, 0.0f, 0.0f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(0.0f, 0.0f, 10.0f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(0.0f, 0.0f, -10.0f), Normal = new Vector3(0.0f, 1.0f, 0.0f) }
        };

        // mask used to make a floor half-transparent
        private readonly MeshVertex[] transparencyMask =
        {
            new MeshVertex { Position = new Vector3(10.0f, 0.0f, 10.0f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-10.0f, 0.0f, 10.0f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-10.0f, 0.0f, -10.0f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(10.0f, 0.0f, -10.0f), Normal = new Vector3(0.0f, 1.0f, 0.0f) }
        };

        // all the needed entities
        Model grid, gridFloor;
        Model[] goal, path;

        HashSet<Logic.PathPlanning.Tree.Node>[] tree;

        public static float time = 0;
        public static bool forward;
        private bool ManipLoaded = false;

        // 3D model
        //Model Crytek;
        public static Model pointMoveable;
        public static Vector2 pointScreen;

        private Model[] Cubes;
        private Model Ground;
        private PhysicsHandler _physics;

        public static Thread MainThread = Thread.CurrentThread;

        public MainWindow(int width, int height, GraphicsMode gMode, string title) : 
            base(width, height, gMode, title, GameWindowFlags.Default, DisplayDevice.Default, 4, 6, GraphicsContextFlags.ForwardCompatible) 
        {
            _physics = new PhysicsHandler();
        }

        protected override void OnLoad(EventArgs e)
        {
            //var unptr = Assimp.Unmanaged.AssimpLibrary.Instance.ImportFile(JointPath, Assimp.PostProcessSteps.None, Assimp.Unmanaged.AssimpLibrary.Instance.CreatePropertyStore());
            //var manptr = Assimp.Scene.FromUnmanagedScene(unptr);
            //var ptr = Assimp.Unmanaged.AssimpLibrary.Instance.ApplyPostProcessing(unptr, Assimp.PostProcessSteps.Triangulate);
            //manptr = Assimp.Scene.FromUnmanagedScene(ptr);

            ShaderHandler.InitializeShaders();

            // defining ImGui controller
            controller = new ImGuiController(Width, Height);

            // Camera is 6 units back and has the proper aspect ratio
            _camera = new Camera((float)(0.75 * Width / Height), new Vector3(-5, 3, 5), -15, -45);

            // workspace grid
            grid = new Model(gridLines, material: MeshMaterial.White);

            gridFloor = new Model(transparencyMask, new uint[] { 1, 0, 3, 1, 2, 3, 1 }, new MeshMaterial
            {
                Diffuse = new Vector4(1.0f, 1.0f, 1.0f, 0.5f)
            });

            pointMoveable = new Model(new MeshVertex[] 
            {
                new MeshVertex { Position = new Vector3(0.0f, 0.0f, 0.0f) }
            }, material: MeshMaterial.Yellow);

            InputHandler.Widget = new AxesWidget(new Axis[3]
            {
                new Axis(Vector4.UnitW, new Vector4(0.3f, 0, 0, 1), new Vector4(1, 0, 0, 1)),
                new Axis(Vector4.UnitW, new Vector4(0, 0.3f, 0, 1), new Vector4(0, 1, 0, 1)),
                new Axis(Vector4.UnitW, new Vector4(0, 0, 0.3f, 1), new Vector4(0, 0, 1, 1))
            }, pointMoveable);

            Cubes = new Model[3];
            for (int i = 0; i < 3; i++)
            {
                Cubes[i] = Primitives.Cube(new MeshMaterial
                {
                    Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
                    Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
                    Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
                    Shininess = 8
                });
            }

            Cubes[0].State.Column3 = new Vector4(0.0f, 3.0f, 0.0f, 1.0f);
            Cubes[1].State.Column3 = new Vector4(0.5f, 4.5f, 0.0f, 1.0f);
            Cubes[2].State.Column3 = new Vector4(1.0f, 6.0f, 0.0f, 1.0f);

            Ground = Primitives.Cube(new MeshMaterial
            {
                Ambient = new Vector4(0.02f, 0.1f, 0.0f, 1.0f),
                Diffuse = new Vector4(0.1f, 0.8f, 0.0f, 1.0f),
                Specular = new Vector4(0.1f, 0.5f, 0.0f, 1.0f),
                Shininess = 8
            });

            Ground.State.M11 *= 10;
            Ground.State.M33 *= 10;
            Ground.State.M22 *= 0.25f;

            base.OnLoad(e);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            // render main part, i.e. workspace
            RenderCore(e);

            // render GUI
            RenderGUI(e);

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

            ShaderHandler.SetupShaders(_camera);

            //pointMoveable.Render(ShaderHandler.GenericShader, () =>
            //{
            //    GL.PointSize(20);
            //    GL.DrawArrays(PrimitiveType.Points, 0, 1);
            //    GL.PointSize(1);
            //});

            foreach (var cube in Cubes)
            {
                cube.Render(ShaderHandler.ComplexShader, MeshMode.Solid | MeshMode.Wireframe | MeshMode.Lighting);
            }

            Ground.Render(ShaderHandler.ComplexShader, MeshMode.Solid | MeshMode.Wireframe | MeshMode.Lighting);

            if (ManipLoaded)
            {
                // render obstacles
                foreach (var obstacle in Manager.Obstacles)
                {
                    obstacle.Render(ShaderHandler.ComplexShader, true);
                }

                for (int i = 0; i < Manager.Manipulators.Length; i++)
                {
                    Manipulator manip = Manager.Manipulators[i];

                    // render manipulator
                    manip.Render(ShaderHandler.ComplexShader);

                    // render goal
                    if (goal[i] != default)
                    {
                        goal[i].Render(ShaderHandler.ComplexShader, MeshMode.Solid, () =>
                        {
                            GL.PointSize(5);
                            GL.DrawArrays(PrimitiveType.Points, 0, 1);
                            GL.PointSize(1);
                            GL.DrawArrays(PrimitiveType.Points, 1, 100);
                        });
                    }

                    // render path
                    if (manip.Path != null && path[i] != null)
                    {
                        int count = manip.Path.Count;
                        path[i].Render(ShaderHandler.ComplexShader, MeshMode.Solid, () =>
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

                            node.Model.Render(ShaderHandler.ComplexShader, MeshMode.Solid, () =>
                            {
                                GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                            });
                        }
                    }
                }
            }

            // workspace grid
            grid.State = Matrix4.Identity;
            grid.Render(ShaderHandler.ComplexShader, MeshMode.Solid, () =>
            {
                GL.DrawArrays(PrimitiveType.Lines, 0, 4);
            });
            for (int i = 1; i < 11; i++)
            {
                grid.State = Matrix4.CreateTranslation(System.Numerics.Vector3.UnitZ * i);
                grid.Render(ShaderHandler.ComplexShader, MeshMode.Solid, () => GL.DrawArrays(PrimitiveType.LineStrip, 0, 2));
                grid.State = Matrix4.CreateTranslation(System.Numerics.Vector3.UnitZ * -i);
                grid.Render(ShaderHandler.ComplexShader, MeshMode.Solid, () => GL.DrawArrays(PrimitiveType.LineStrip, 0, 2));

                grid.State = Matrix4.CreateTranslation(System.Numerics.Vector3.UnitX * i);
                grid.Render(ShaderHandler.ComplexShader, MeshMode.Solid, () => GL.DrawArrays(PrimitiveType.LineStrip, 2, 2));
                grid.State = Matrix4.CreateTranslation(System.Numerics.Vector3.UnitX * -i);
                grid.Render(ShaderHandler.ComplexShader, MeshMode.Solid, () => GL.DrawArrays(PrimitiveType.LineStrip, 2, 2));
            }

            gridFloor.Render(ShaderHandler.ComplexShader, MeshMode.Solid, () =>  // TODO: all help should be placed in a separate document (aka documentation)
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

            InputHandler.Widget.Render(ShaderHandler.ComplexShader, () =>
            {
                GL.Disable(EnableCap.DepthTest);
                GL.DrawArrays(PrimitiveType.Lines, 0, 2);
                GL.Enable(EnableCap.DepthTest);
            });

            //ImGui.ShowDemoWindow();

            base.OnRenderFrame(e);
        }

        protected void RenderGUI(FrameEventArgs e)
        {
            // update GUI controller
            controller.Update(this, (float)e.Time);

            // GUI viewport
            GL.Viewport(0, 0, Width, Height);

            // clear viewport
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
                                var jointModel = new Model(InputHandler.JointPath);
                                var linkModel = new Model(InputHandler.LinkPath);
                                var gripperModel = new Model(InputHandler.GripperPath);

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

                                //Crytek = new Model(InputHandler.NanosuitPath);

                                // wait for loading process to finish
                                Dispatcher.ActionsDone.Reset();
                                Dispatcher.ActionsDone.WaitOne();

                                // update workspace with newly loaded model
                                UpdateWorkspace();
                                Dispatcher.UpdateThreads();
                                ManipLoaded = true;

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
                    // inform the program that the window capture has been queried
                    InputHandler.Capture = true;
                }
                if (ImGui.IsItemHovered())
                {
                    ImGui.SameLine();
                    ImGui.TextWrapped("Takes a picture of the entire window");
                }

                // save path for captured screenshot
                ImGui.InputText("Save path", ref InputHandler.ScreenshotsPath, 100);
                InputHandler.TextIsEdited = ImGui.IsItemActive();

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

        private OpenTK.Matrix4 Convert(BulletSharp.Math.Matrix m)
        {
            return OpenTK.Matrix4.Transpose(new OpenTK.Matrix4(
                m.M11, m.M12, m.M13, m.M14,
                m.M21, m.M22, m.M23, m.M24,
                m.M31, m.M32, m.M33, m.M34,
                m.M41, m.M42, m.M43, m.M44));
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            // update physics controller
            _physics.Update((float)e.Time);

            var monitoredBody = (RigidBody)_physics.World.CollisionObjectArray[0];
            object context = "context";
            _physics.World.ContactPairTest(
                    _physics.World.CollisionObjectArray[1],
                    _physics.World.CollisionObjectArray[0],
                    new CollisionCallback(monitoredBody, context));

            // process physics
            foreach (RigidBody body in _physics.World.CollisionObjectArray)
            {
                if (!"Ground".Equals(body.UserObject))
                    Cubes[body.UserIndex].State = Convert(body.WorldTransform);
            }

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
                        data.AddRange(Primitives.SpherePointCloud(goalAttr.Radius, goalAttr.Center, 100));
                        goal[i] = new Model(MeshVertex.Convert(data), material: MeshMaterial.Yellow);
                    }

                    // obtained path
                    if (manip.Path != null)
                    {
                        // path may change at any time in control thread; GetRange() guarantees thread sync
                        // TODO: but not for the case when path shortens; fix!
                        int count = manip.Path.Count;
                        var pathRange = MeshVertex.Convert(manip.Path.GetRange(0, count));
                        if (path[i] == default)
                        {
                            path[i] = new Model(pathRange, material: MeshMaterial.Red);
                        }
                        else
                        {
                            path[i].Update(0, pathRange, new uint[0]);
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
            // TODO: create a list that holds references to all models in the scene, and dispose here all models in that list

            // freeing all the used resources
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindVertexArray(0);
            GL.UseProgram(0);

            ShaderHandler.DeleteShaders();

            _physics.ExitPhysics();

            base.OnUnload(e);
        }

        protected void UpdateWorkspace()  // TODO: move somewhere else
        {
            // initializing manager
            Manager.Initialize();

            // initializing all displaying entities
            int manip_length = Manager.Manipulators.Length;
            goal = new Model[manip_length];
            path = new Model[manip_length];
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
        public static Model CreateTreeBranch(System.Numerics.Vector3 p1, System.Numerics.Vector3 p2)
        {
            return new Model(new MeshVertex[] 
            { 
                new MeshVertex { Position = new Vector3(p1.X, p1.Y, p1.Z) },
                new MeshVertex { Position = new Vector3(p2.X, p2.Y, p2.Z) }
            }, material: MeshMaterial.Black);
        }

        protected override void OnKeyPress(KeyPressEventArgs e)
        {
            controller.PressChar(e.KeyChar);

            base.OnKeyPress(e);            
        }
    }
}