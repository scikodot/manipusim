using System;
using System.Linq;

using OpenTK;
using OpenTK.Graphics.OpenGL4;
using ImGuiNET;

using Logic;
using Logic.InverseKinematics;
using Logic.PathPlanning;
using Physics;

namespace Graphics
{
    class MainWindowImGui : ImGuiHandler
    {
        private const ImGuiTreeNodeFlags _baseTreeNodeFlags = ImGuiTreeNodeFlags.OpenOnArrow;
        private const ImGuiTreeNodeFlags _baseTreeLeafFlags = ImGuiTreeNodeFlags.Leaf | ImGuiTreeNodeFlags.NoTreePushOnOpen;

        private static bool _swapPropertiesWindows;

        public MainWindowImGui(MainWindow mainWindow) : base(mainWindow) { }

        public void Render(FrameEventArgs e)
        {
            // update GUI
            Update((float)e.Time);

            //ImGui.ShowDemoWindow();

            // GUI viewport
            GL.Viewport(0, 0, Window.Width, Window.Height);

            // clear viewport
            GL.Enable(EnableCap.ScissorTest);
            GL.Scissor(0, 0, (int)(0.25 * Window.Width), Window.Height);
            GL.ClearColor(0.3f, 0.3f, 0.3f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            GL.Disable(EnableCap.ScissorTest);

            // render all the necessary windows
            RenderMenu();
            RenderManipulatorsWindow();
            RenderObstaclesWindow();
            RenderOptionsWindow();

            if (MainWindow.Mode == InteractionMode.Design)
            {
                RenderPropertiesWindow();
            }

            // render controller and check for errors
            Render();
            Util.CheckGLError("End of frame");
        }

        #region MAIN_MENU_BAR
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

                    if (ImGui.MenuItem("BenchmarkIK"))
                    {
                        Benchmark.RunInverseKinematics();
                    }

                    if (ImGui.MenuItem("BenchmarkPP"))
                    {
                        Benchmark.RunPathPlanning();
                    }

                    ImGui.EndMenu();
                }

                ImGui.EndMainMenuBar();
            }
        }
        #endregion

        #region MANIPULATORS_WINDOW
        private void RenderManipulatorsWindow()
        {
            // manipulators window
            if (ImGui.Begin("Manipulators",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize |
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, 19));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Window.Width - 2), (int)(0.375 * (Window.Height - 19))));

                if (ImGui.Button("Create"))
                {
                    ImGui.OpenPopup("ManipulatorCreate");
                }

                if (ImGui.BeginPopup("ManipulatorCreate"))
                {
                    ImGui.Text("Links number");
                    ImGui.InputInt("", ref ManipulatorHandler.DefaultLinksNumber, 0, 0);

                    if (ImGui.Button("Create"))
                    {
                        MainWindow.CreateDefaultManipulator(ManipulatorHandler.DefaultLinksNumber);

                        ImGui.CloseCurrentPopup();
                    }

                    ImGui.EndPopup();
                }

                ImGui.Separator();

                ImGui.PushStyleVar(ImGuiStyleVar.ChildRounding, 5);
                ImGui.BeginChild("ManipulatorList", ImGui.GetContentRegionAvail(), true);

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
        #endregion

        #region OBSTACLES_WINDOW
        private void RenderObstaclesWindow()
        {
            // obstacles window
            if (ImGui.Begin("Obstacles",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize |
                ImGuiWindowFlags.HorizontalScrollbar))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.375 * (Window.Height - 19) + 19)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Window.Width - 2), (int)(0.375 * (Window.Height - 19))));

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
                            ImGui.CloseCurrentPopup();
                        }
                    }

                    ImGui.EndPopup();
                }

                ImGui.Separator();

                ImGui.PushStyleVar(ImGuiStyleVar.ChildRounding, 5);
                ImGui.BeginChild("ObstacleList", ImGui.GetContentRegionAvail(), true);

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
        #endregion

        #region OPTIONS_WINDOW
        private void RenderOptionsWindow()
        {
            // options and info window
            if (ImGui.Begin("Options & Info",
                ImGuiWindowFlags.NoCollapse |
                ImGuiWindowFlags.NoMove |
                ImGuiWindowFlags.NoResize))
            {
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.75 * (Window.Height - 19) + 19)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Window.Width - 2), (int)(0.25 * (Window.Height - 19))));

                string targetMode;
                switch (MainWindow.Mode)
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
                    MainWindow.SwitchMode();
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
                ImGui.SetCursorScreenPos(new System.Numerics.Vector2(8, Window.Height - 8 - ImGui.CalcTextSize("Framerate:").Y));
                ImGui.Text(string.Format("Framerate: {0:F1} FPS", ImGui.GetIO().Framerate));

                ImGui.End();
            }
        }
        #endregion

        #region PROPERTIES_WINDOW
        private void RenderPropertiesWindow()
        {
            if (InputHandler.CurrentSelectedObject is Manipulator manipulator)
            {
                RenderPropertiesWindowTemplate("Manipulator properties", manipulator, ManipulatorProperties);
            }
            else if (InputHandler.CurrentSelectedObject is Joint joint)
            {
                RenderPropertiesWindowTemplate("Joint properties", joint, JointProperties);
            }
            else if (InputHandler.CurrentSelectedObject is Link link)
            {
                RenderPropertiesWindowTemplate("Link properties", link, LinkProperties);
            }
            else if (InputHandler.CurrentSelectedObject is Obstacle obstacle)
            {
                RenderPropertiesWindowTemplate("Obstacle properties", obstacle, ObstacleProperties);
            }
        }

        private void RenderPropertiesWindowTemplate<T>(string title, T selectable, Action<T> renderProperties) where T : ISelectable
        {
            if (ImGui.Begin(title/* + (changed ? "##first" : "##last")*/,
                    ImGuiWindowFlags.NoCollapse |
                    ImGuiWindowFlags.NoMove |
                    ImGuiWindowFlags.NoResize |
                    ImGuiWindowFlags.HorizontalScrollbar))
            {
                // swap window on the ID stack to not inherit fields inputs from the previous objects
                ImGui.PushID(_swapPropertiesWindows ? 1 : 0);

                // set position and size of the window
                ImGui.SetWindowPos(new System.Numerics.Vector2((int)(0.25 * Window.Width), (int)(0.75 * Window.Height)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Window.Width - 2), (int)(0.25 * Window.Height)));

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

            if (ImGui.BeginTabBar("ManipulatorTabs"))
            {
                if (ImGui.BeginTabItem("Solver"))
                {
                    int currType = (int)manipulator.Controller.InverseKinematicsSolver.Type;  // TODO: refactor?
                    int newType = currType;
                    ImGui.Combo("Type", ref newType, InverseKinematicsSolver.Types, InverseKinematicsSolver.Types.Length);

                    // change inverse kinematics solver if queried
                    if (newType != currType)
                    {
                        switch ((InverseKinematicsSolverType)newType)
                        {
                            case InverseKinematicsSolverType.JacobianTranspose:
                                manipulator.Controller.InverseKinematicsSolver = JacobianTranspose.Default();
                                break;
                            case InverseKinematicsSolverType.JacobianPseudoinverse:
                                manipulator.Controller.InverseKinematicsSolver = JacobianPseudoinverse.Default();
                                break;
                            case InverseKinematicsSolverType.DampedLeastSquares:
                                manipulator.Controller.InverseKinematicsSolver = DampedLeastSquares.Default();
                                break;
                        }
                    }

                    ImGui.Separator();

                    // inverse kinematics solver properties
                    ImGui.InputInt("Max iterations", ref manipulator.Controller.InverseKinematicsSolver.MaxIterations);

                    if (manipulator.Controller.InverseKinematicsSolver is JacobianTranspose jacobianTranspose)
                    {
                        ImGui.InputFloat("Base damping coefficient", ref jacobianTranspose.Damping);
                    }
                    else if (manipulator.Controller.InverseKinematicsSolver is JacobianPseudoinverse jacobianInverse)
                    {
                        // TODO: input something here?
                    }
                    else if (manipulator.Controller.InverseKinematicsSolver is DampedLeastSquares dampedLeastSquares)
                    {
                        ImGui.InputFloat("Damping coefficient", ref dampedLeastSquares.Damping);
                    }

                    ImGui.EndTabItem();
                }

                if (ImGui.BeginTabItem("Planner"))
                {
                    int currType = (int)manipulator.Controller.PathPlanner.Type;
                    int newType = currType;
                    ImGui.Combo("Type", ref newType, PathPlanner.Types, PathPlanner.Types.Length);

                    // change path planner if queried
                    if (newType != currType)
                    {
                        switch ((PathPlannerType)newType)
                        {
                            case PathPlannerType.RRT:
                                manipulator.Controller.PathPlanner = RRT.Default();
                                break;
                            case PathPlannerType.ARRT:
                                manipulator.Controller.PathPlanner = ARRT.Default(manipulator);
                                break;
                            case PathPlannerType.GeneticAlgorithm:
                                manipulator.Controller.PathPlanner = GeneticAlgorithm.Default();
                                break;
                        }
                    }

                    ImGui.Separator();

                    // path planner properties
                    ImGui.Checkbox("Collision check", ref manipulator.Controller.PathPlanner.CollisionCheck);
                    ImGui.InputInt("Max iterations", ref manipulator.Controller.PathPlanner.MaxIterations);

                    if (manipulator.Controller.PathPlanner is RRT rrt)
                    {
                        ImGui.Text($"Tree size: {(rrt.Tree == null ? 0 : rrt.Tree.Count)} nodes");  // TODO: move to Statistics window
                        ImGui.Checkbox($"Show tree", ref rrt.ShowTree);
                        ImGui.Checkbox("Discard outliers", ref rrt.DiscardOutliers);
                        ImGui.InputFloat("Step", ref rrt.Step);
                        ImGui.InputFloat("Threshold", ref rrt.Threshold);

                        if (rrt is ARRT arrt)
                        {
                            ImGui.InputInt("Attractors count", ref arrt.AttractorsCount);
                            ImGui.InputInt("Trim period", ref arrt.TrimPeriod);
                        }
                    }
                    else if (manipulator.Controller.PathPlanner is GeneticAlgorithm geneticAlgorithm)
                    {
                        // TODO: add genetic algorithm related properties

                    }

                    ImGui.EndTabItem();
                }

                if (ImGui.BeginTabItem("Controller"))
                {
                    // TODO: add motion control related properties

                    ImGui.EndTabItem();
                }

                ImGui.EndTabBar();
            }
        }

        private void JointProperties(Joint joint)
        {
            ImGui.Checkbox("Activate", ref joint.Active);  // TODO: for debug use only

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
                    // TODO: add model related properties

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

        private void SwapPropertiesWindows()
        {
            _swapPropertiesWindows = !_swapPropertiesWindows;
        }
        #endregion

        #region SELECTION
        public void OnSelectedObjectChanged(object sender, EventArgs e)
        {
            SwapPropertiesWindows();
        }

        private void UpdateSelection(object selected)
        {
            InputHandler.ClearSelection();

            if (selected == InputHandler.CurrentSelectedObject)
            {
                InputHandler.CurrentSelectedObject = null;
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

                InputHandler.CurrentSelectedObject = selected;
                SwapPropertiesWindows();
            }
        }

        private ImGuiTreeNodeFlags GetTreeNodeSelectionFlag(object selected)
        {
            return selected == InputHandler.CurrentSelectedObject ? ImGuiTreeNodeFlags.Selected : ImGuiTreeNodeFlags.None;
        }
        #endregion
    }
}
