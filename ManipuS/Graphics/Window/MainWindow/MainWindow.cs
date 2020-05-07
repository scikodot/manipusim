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

namespace Graphics
{
    public enum InteractionModes
    {
        Design,
        Animate,
        ToDesign,
        ToAnimate
    }

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
        private Model grid, gridFloor;

        private readonly List<TreeModel> trees = new List<TreeModel>();
        private readonly List<PathModel> paths = new List<PathModel>();
        private readonly List<Model> goals = new List<Model>();

        public static float time = 0;
        public static bool forward;
        private bool ManipLoaded = false;

        // 3D model
        //Model Crytek;
        public static Model pointMoveable;
        public static Vector2 pointScreen;

        private Obstacle[] Cubes;
        private Obstacle Ground;
        private Obstacle Sphere;
        private Obstacle Cylinder;

        public static Thread MainThread = Thread.CurrentThread;
        public InteractionModes Mode = InteractionModes.Design;

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

            InputHandler.Widget = new AxesWidget(new AxesWidget.Axis[3]
            {
                new AxesWidget.Axis(Vector4.UnitW, new Vector4(0.3f, 0, 0, 1), new Vector4(1, 0, 0, 1)),
                new AxesWidget.Axis(Vector4.UnitW, new Vector4(0, 0.3f, 0, 1), new Vector4(0, 1, 0, 1)),
                new AxesWidget.Axis(Vector4.UnitW, new Vector4(0, 0, 0.3f, 1), new Vector4(0, 0, 1, 1))
            }, pointMoveable);

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

            Ground = new Obstacle(Primitives.Cube(5, 0.2f, 5, new MeshMaterial
            {
                Ambient = new Vector4(0.02f, 0.1f, 0.0f, 1.0f),
                Diffuse = new Vector4(0.0f, 0.6f, 0.8f, 1.0f),
                Specular = new Vector4(0.5f, 0.1f, 0.0f, 1.0f),
                Shininess = 8
            }), PhysicsHandler.CreateStaticCollider(new BoxShape(5, 0.2f, 5)));

            Sphere = new Obstacle(Primitives.Sphere(1, 100, 100, new MeshMaterial
            {
                Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
                Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
                Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
                Shininess = 8
            }), PhysicsHandler.CreateDynamicCollider(new SphereShape(1), 1, Matrix.Translation(-3, 3, -3)));

            Cylinder = new Obstacle(Primitives.Cylinder(0.25f, 1, 1, 50, new MeshMaterial
            {
                Ambient = new Vector4(0.1f, 0.1f, 0.0f, 1.0f),
                Diffuse = new Vector4(0.8f, 0.8f, 0.0f, 1.0f),
                Specular = new Vector4(0.5f, 0.5f, 0.0f, 1.0f),
                Shininess = 8
            }), PhysicsHandler.CreateDynamicCollider(new CylinderShape(0.25f, 1, 0.25f), 1, Matrix.Translation(3, 4, -3)));

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

            //pointMoveable.Render(ShaderHandler.ComplexShader, MeshMode.Solid, () =>
            //{
            //    GL.PointSize(20);
            //    GL.DrawArrays(PrimitiveType.Points, 0, 1);
            //    GL.PointSize(1);
            //});

            ObstacleHandler.RenderAll(ShaderHandler.ComplexShader);

            //foreach (var cube in Cubes)
            //{
            //    cube.Render(ShaderHandler.ComplexShader, MeshMode.Solid | MeshMode.Wireframe | MeshMode.Lighting);
            //}

            //Ground.Render(ShaderHandler.ComplexShader, MeshMode.Solid | MeshMode.Wireframe | MeshMode.Lighting);

            //Sphere.Render(ShaderHandler.ComplexShader, MeshMode.Solid | MeshMode.Lighting);

            //Cylinder.Render(ShaderHandler.ComplexShader, MeshMode.Solid | MeshMode.Lighting);

            //ShaderHandler.GenericShader.Use();
            //var model = OpenTK.Matrix4.Identity;
            //ShaderHandler.GenericShader.SetMatrix4("model", ref model);

            if (ManipLoaded)
            {
                // render obstacles
                foreach (var obstacle in ObstacleHandler.Obstacles)
                {
                    obstacle.Render(ShaderHandler.ComplexShader, MeshMode.Solid | MeshMode.Lighting);
                }

                for (int i = 0; i < ManipHandler.Count; i++)
                {
                    Manipulator manip = ManipHandler.Manipulators[i];

                    // render manipulator
                    manip.Render(ShaderHandler.ComplexShader);

                    // render goal
                    if (goals[i] != default)
                    {
                        goals[i].Render(ShaderHandler.ComplexShader, MeshMode.Solid);
                    }

                    // render path
                    if (manip.Path != null)
                    {
                        paths[i].Render(ShaderHandler.ComplexShader, MeshMode.Solid);
                    }

                    // render tree
                    if (manip.ShowTree)
                    {
                        trees[i].Render(ShaderHandler.ComplexShader, MeshMode.Solid);
                    }
                }
            }

            // TODO: render selections, i.e. highlights of the currently selected objects;
            // Tips for GUI:
            // - if there's only one selected object, render a properties tab with that object's properties
            // --- obstacle properties: type, position, orientation and unique properties that define dimensions
            // --- joint properties: coordinate and its range, position and axis (position can be changed directly only for manipulators with straight links)
            // --- links properties are not specified for straight links, because they just connect adjacent joints; for arbitrarily formed links, some future research is needed
            // - if there are multiple objects, only their positions and orientations can be changed (relatively, not absolutely), e.g. with widgets
            // - only a group of common objects can be selected, i.e. obstacles/obstacles, manipulators/manipulators, joints/joints, links/links, etc.

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

            base.OnRenderFrame(e);
        }

        protected void RenderGUI(FrameEventArgs e)
        {
            // update GUI controller
            controller.Update(this, (float)e.Time);

            ImGui.ShowDemoWindow();

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
                                //// load components' models
                                //var jointModel = new Model(InputHandler.JointPath);
                                //var linkModel = new Model(InputHandler.LinkPath);
                                //var gripperModel = new Model(InputHandler.GripperPath);

                                //var MB = WorkspaceBuffer.ManipBuffer;
                                //for (int i = 0; i < MB.Length; i++)
                                //{
                                //    for (int j = 0; j < MB[i].N; j++)
                                //    {
                                //        MB[i].Links[j].Model = linkModel.ShallowCopy();
                                //        MB[i].Links[j].Collider = PhysicsHandler.CreateKinematicCollider(new CylinderShape(0.15f, 1, 0.15f));
                                //        MB[i].Joints[j].Model = jointModel.ShallowCopy();
                                //        MB[i].Joints[j].Collider = PhysicsHandler.CreateKinematicCollider(new SphereShape(0.2f));
                                //    }

                                //    MB[i].Joints[MB[i].N].Model = gripperModel;
                                //    MB[i].Joints[MB[i].N].Collider = PhysicsHandler.CreateKinematicCollider(new SphereShape(0.2f));
                                //}

                                ////Crytek = new Model(InputHandler.NanosuitPath);

                                //// wait for loading process to finish
                                //Dispatcher.ActionsDone.Reset();
                                //Dispatcher.ActionsDone.WaitOne();

                                //// update scene
                                //UpdateScene();
                                //ManipLoaded = true;

                                ////Dispatcher.RunObstacles();
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

                if (ImGui.Button("Create"))
                {
                    // TODO: create the default manipulator

                    Dispatcher.ActiveTasks.Add(Task.Run(() =>
                    {
                        // load components' models
                        var jointModel = new Model(InputHandler.JointPath);
                        var linkModel = new Model(InputHandler.LinkPath);
                        var gripperModel = new Model(InputHandler.GripperPath);

                        var MB = WorkspaceBuffer.ManipBuffer;
                        for (int j = 0; j < MB[0].N; j++)
                        {
                            MB[0].Links[j].Model = linkModel.ShallowCopy();
                            MB[0].Links[j].Collider = PhysicsHandler.CreateKinematicCollider(new CylinderShape(0.15f, 1, 0.15f));
                            MB[0].Joints[j].Model = jointModel.ShallowCopy();
                            MB[0].Joints[j].Collider = PhysicsHandler.CreateKinematicCollider(new SphereShape(0.2f));
                        }

                        MB[0].Joints[MB[0].N].Model = gripperModel;
                        MB[0].Joints[MB[0].N].Collider = PhysicsHandler.CreateKinematicCollider(new SphereShape(0.2f));

                        var manip = new Manipulator(MB[0]);
                        ManipHandler.Add(manip);

                        // create a new model for the manipulator goal, path and tree
                        trees.Add(new TreeModel(10000, MeshMaterial.Black));
                        paths.Add(new PathModel(10000, MeshMaterial.Red));
                        goals.Add(Primitives.Sphere(0.05f, 5, 5, MeshMaterial.Yellow, Matrix4.CreateTranslation(manip.Goal)));

                        // wait for loading process to finish
                        Dispatcher.ActionsDone.Reset();
                        Dispatcher.ActionsDone.WaitOne();

                        // update workspace with newly loaded model
                        UpdateScene();
                        ManipLoaded = true;
                    }));
                }

                ImGui.Separator();

                ImGui.PushStyleVar(ImGuiStyleVar.ChildRounding, 5);
                ImGui.BeginChild("Obstacles' list", new System.Numerics.Vector2(ImGui.GetWindowContentRegionWidth(), 138), true);

                if (ManipHandler.Count != 0)
                {
                    var MB = WorkspaceBuffer.ManipBuffer;
                    for (int j = 0; j < ManipHandler.Count; j++)
                    {
                        var manip = ManipHandler.Manipulators[j];
                        if (ImGui.TreeNode($"Manip {j}"))
                        {
                            ImGui.Text($"Time spent: {manip.Controller.Timer.ElapsedMilliseconds / 1000.0f} s");

                            int treeCount = manip.Tree == null ? 0 : manip.Tree.Count;
                            ImGui.Checkbox($"Show tree ({treeCount} verts)", ref manip.ShowTree);

                            ImGui.Checkbox($"Show collider", ref manip.ShowCollider);

                            ImGui.InputFloat3("Goal", ref manip.Goal);
                            //ImGui.InputInt("Links number", ref MB[j].N);

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
                                for (int i = 0; i < manip.Joints.Length; i++)
                                {
                                    if (ImGui.TreeNode(i == 0 ? "Base" : $"Joint {i}"))
                                    {
                                        ImGui.InputFloat3("Axis", ref manip.Joints[i].InitialAxis);
                                        ImGui.InputFloat3("Position", ref manip.Joints[i].InitialPosition);
                                        ImGui.InputFloat("Initial GC (deg)", ref manip.Joints[i].InitialCoordinate);
                                        ImGui.InputFloat2("GC range", ref manip.Joints[i].CoordinateRange);
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

                ImGui.EndChild();

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

                if (ImGui.Button("Create"))
                {
                    ImGui.OpenPopup("obstCreate");
                }

                if (ImGui.BeginPopup("obstCreate"))
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
                ImGui.BeginChild("Obstacles' list", new System.Numerics.Vector2(ImGui.GetWindowContentRegionWidth(), 138), true);

                if (ObstacleHandler.Count != 0)
                {
                    for (int i = 0; i < ObstacleHandler.Count; i++)
                    {
                        var obst = ObstacleHandler.Obstacles[i];
                        if (ImGui.TreeNode($"Obst {i}"))
                        {
                            ImGui.Text($"Shape type: {obst.ShapeType}");

                            int type = (int)obst.Type;
                            ImGui.Combo("Physics type", ref type, PhysicsHandler.RigidBodyTypes, PhysicsHandler.RigidBodyTypes.Length);
                            obst.Type = (RigidBodyType)type;

                            ImGui.Checkbox("Show collider", ref obst.ShowCollider);

                            ImGui.InputFloat3("Orientation", ref obst.Orientation);

                            ImGui.InputFloat3("Translation", ref obst.Translation);

                            switch (obst.ShapeType)  // TODO: handle zero cases; when the dimensions are zeroed, objects disappear!!!
                            {
                                case BroadphaseNativeType.BoxShape:
                                    ImGui.InputFloat3("Half extents", ref (obst.Collider as BoxCollider).Size);
                                    break;
                                case BroadphaseNativeType.SphereShape:
                                    ImGui.InputFloat("Radius", ref (obst.Collider as SphereCollider).Radius);
                                    break;
                                case BroadphaseNativeType.CylinderShape:
                                    ImGui.InputFloat("Radius", ref (obst.Collider as CylinderCollider).Radius);
                                    ImGui.InputFloat("Length", ref (obst.Collider as CylinderCollider).Length);
                                    break;
                            }

                            ImGui.TreePop();
                        }
                    }
                }
                else
                {
                    ImGui.Text("No obstacles at the scene.");
                }

                ImGui.EndChild();

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
                    ref WorkspaceBuffer.InverseKinematicsBuffer.InverseKinematicsSolverID,
                    Logic.InverseKinematics.IKSolver.Types,
                    Logic.InverseKinematics.IKSolver.Types.Length);
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

            // options & info window
            if (ImGui.Begin("Options & Info",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.75 * Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Width - 2), (int)(0.25 * Height)));

                if (ImGui.Button("Animate"))
                {
                    Mode = InteractionModes.ToAnimate;
                }
                if (ImGui.IsItemHovered())
                {
                    ImGui.SameLine();
                    ImGui.TextWrapped("Runs path searching process");
                }

                if (ImGui.Button("Design"))
                {
                    Mode = InteractionModes.ToDesign;
                }
                if (ImGui.IsItemHovered())
                {
                    ImGui.SameLine();
                    ImGui.TextWrapped("Stops path searching process");
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
                        // update scene
                        UpdateScene();

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

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            // update physics controller
            PhysicsHandler.Update((float)e.Time);

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
                case InteractionModes.Design:

                    ObstacleHandler.DesignUpdate();

                    break;
                case InteractionModes.Animate:

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

                    for (int i = 0; i < ManipHandler.Count; i++)
                    {
                        Manipulator manip = ManipHandler.Manipulators[i];

                        // manipulator's path
                        if (manip.Path != null)
                        {
                            // move along the path
                            manip.FollowPath();

                            // update path model state
                            paths[i].Update(i);
                        }

                        // random tree
                        if (manip.Tree != null)
                        {
                            // update tree model state
                            trees[i].Update(i);
                        }
                    }

                    break;
                case InteractionModes.ToDesign:

                    // convert all obstacles to kinematic type to allow free displacement
                    ObstacleHandler.ToDesign();

                    ManipHandler.ToDesign();

                    // update workspace
                    UpdateScene();

                    Mode = InteractionModes.Design;

                    break;
                case InteractionModes.ToAnimate:

                    // convert all obstacles to their native physics types
                    ObstacleHandler.ToAnimate();

                    // run threads for all manipulators
                    ManipHandler.ToAnimate();

                    Mode = InteractionModes.Animate;

                    break;
            }

            base.OnUpdateFrame(e);
        }

        protected void UpdateScene()
        {
            // reset trees and paths
            trees.ForEach(x => x.Reset());
            paths.ForEach(x => x.Reset());
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

            PhysicsHandler.ExitPhysics();

            base.OnUnload(e);
        }

        protected override void OnKeyPress(KeyPressEventArgs e)
        {
            controller.PressChar(e.KeyChar);

            base.OnKeyPress(e);            
        }
    }
}