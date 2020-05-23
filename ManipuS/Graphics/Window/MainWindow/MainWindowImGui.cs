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
        private static bool _swapPropertiesWindows;
        private static ImGuiTreeNodeFlags _baseTreeNodeFlags = ImGuiTreeNodeFlags.OpenOnArrow;
        private static ImGuiTreeNodeFlags _baseTreeLeafFlags = ImGuiTreeNodeFlags.Leaf | ImGuiTreeNodeFlags.NoTreePushOnOpen;

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

        public void OnSelectedObjectChanged(object sender, EventArgs e)
        {
            SwapPropertiesWindows();
        }

        private void SwapPropertiesWindows()
        {
            _swapPropertiesWindows = !_swapPropertiesWindows;
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
                        (Window as MainWindow).CreateDefaultManipulator(ManipulatorHandler.DefaultLinksNumber);
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
                    MainWindow.ChangeMode();
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

        private void RenderPropertiesWindow()
        {
            if (InputHandler.CurrentSelectedObject is Manipulator manipulator)
            {
                RenderPropertiesWindowTemplate("Manipulator properties", ManipulatorProperties, manipulator);
            }
            else if (InputHandler.CurrentSelectedObject is Joint joint)
            {
                RenderPropertiesWindowTemplate("Joint properties", JointProperties, joint);
            }
            else if (InputHandler.CurrentSelectedObject is Link link)
            {
                RenderPropertiesWindowTemplate("Link properties", LinkProperties, link);
            }
            else if (InputHandler.CurrentSelectedObject is Obstacle obstacle)
            {
                RenderPropertiesWindowTemplate("Obstacle properties", ObstacleProperties, obstacle);
            }
        }

        private void RenderPropertiesWindowTemplate<T>(string title, Action<T> renderProperties, T selectable) where T : ISelectable
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
                        manipulator.Controller.PlanSolver = new JacobianPseudoinverse(IB.Precision, IB.StepSize, IB.MaxTime);
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
            else if (manipulator.Controller.PlanSolver is JacobianPseudoinverse jacobianInverse)
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
                        manipulator.Controller.PathPlanner = new ARRT(manipulator, PB.k, false, PB.d, 5000, PB.k / 10);
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
                ImGui.Checkbox($"Show tree", ref manipulator.ShowTree);  // TODO: all tree properties should be in path planner!
                ImGui.Checkbox("Discard outliers", ref rrt.DiscardOutliers);
                ImGui.InputFloat("Step", ref rrt.Step);
                ImGui.InputFloat("Threshold", ref rrt.Threshold);

                if (manipulator.Controller.PathPlanner is ARRT arrt)
                {
                    ImGui.InputInt("Attractors count", ref arrt.AttractorsCount);
                    ImGui.InputInt("Trim period", ref arrt.TrimPeriod);
                }
            }
            else if (manipulator.Controller.PathPlanner is GeneticAlgorithm geneticAlgorithm)
            {
                // TODO: add properties
            }

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
    }
}
