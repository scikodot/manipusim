using System;
using System.Linq;

using OpenTK.Windowing.Common;
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

        private bool _swapPropertiesWindows;
        private int _selectedPathIndex = -1;
        private Path.Node _selectedPoint;
        private int _selectedStatIndex = -1;

        private readonly MainWindow _parent;

        public MainWindowImGui(MainWindow mainWindow) : base(mainWindow)
        {
            _parent = mainWindow;
        }

        public void Render(FrameEventArgs e)
        {
            // update GUI
            Update((float)e.Time);

            //ImGui.ShowDemoWindow();

            // GUI viewport
            GL.Viewport(0, 0, Window.Size.X, Window.Size.Y);

            // clear viewport
            GL.Enable(EnableCap.ScissorTest);
            GL.Scissor(0, 0, (int)(0.25 * Window.Size.X), Window.Size.Y);
            GL.ClearColor(0.3f, 0.3f, 0.3f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            GL.Disable(EnableCap.ScissorTest);

            // render all the necessary windows
            //RenderMenu();
            RenderManipulatorsWindow();
            RenderObstaclesWindow();
            //RenderOptionsWindow();

            if (_parent.InputHandler.InteractionMode == InteractionMode.Design)
            {
                RenderPropertiesWindow();
            }
            else
            {
                //RenderStatisticsWindow();
            }

            // render controller and check for errors
            Render();
            Util.CheckGLError("End of frame");
        }

        private void InputElement()
        {

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

                    ImGui.EndMenu();
                }

                if (ImGui.BeginMenu("Edit"))
                {

                    ImGui.EndMenu();
                }

                if (ImGui.BeginMenu("View"))
                {


                    ImGui.EndMenu();
                }

                if (ImGui.BeginMenu("Extra"))
                {
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
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Window.Size.X - 2), (int)(0.5 * (Window.Size.Y - 19))));

                if (ImGui.Button("Create"))
                {
                    ImGui.OpenPopup("ManipulatorCreate");
                }

                ImGui.SameLine();

                if (ImGui.Button("Remove"))
                {
                    if (_parent.InputHandler.SelectedObject is Manipulator manipulator)
                    {
                        _parent.ManipulatorHandler.Remove(manipulator);
                    }
                }

                if (ImGui.BeginPopup("ManipulatorCreate"))
                {
                    int linksNumber = _parent.ManipulatorHandler.DefaultLinksNumber;
                    ImGui.InputInt("Links number", ref linksNumber);

                    float linksLength = _parent.ManipulatorHandler.DefaultLinksLength;
                    ImGui.InputFloat("Links length", ref linksLength);

                    if (linksNumber < 2 || linksLength <= 0)
                    {
                        // TODO: handle unallowed cases
                    }
                    else
                    {
                        // memoize parameters
                        _parent.ManipulatorHandler.DefaultLinksNumber = linksNumber;
                        _parent.ManipulatorHandler.DefaultLinksLength = linksLength;

                        if (ImGui.Button("Create"))
                        {
                            _parent.CreateDefaultManipulator();

                            ImGui.CloseCurrentPopup();
                        }
                    }

                    ImGui.EndPopup();
                }

                ImGui.Separator();

                ImGui.PushStyleVar(ImGuiStyleVar.ChildRounding, 5);
                if (ImGui.BeginChild("ManipulatorList", ImGui.GetContentRegionAvail(), true))
                {
                    if (_parent.ManipulatorHandler.Count != 0)
                    {
                        for (int i = 0; i < _parent.ManipulatorHandler.Count; i++)
                        {
                            var manipulator = _parent.ManipulatorHandler.Manipulators[i];
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
                            }
                        }
                    }
                    else
                    {
                        ImGui.Text("Empty.");
                    }

                    ImGui.EndChild();
                }

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
                ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.5 * (Window.Size.Y - 19) + 19)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Window.Size.X - 2), (int)(0.5 * (Window.Size.Y - 19))));

                if (ImGui.Button("Create"))
                {
                    ImGui.OpenPopup("ObstacleCreate");
                }

                ImGui.SameLine();

                if (ImGui.Button("Remove"))
                {
                    if (_parent.InputHandler.SelectedObject is Obstacle obstacle)
                    {
                        _parent.ObstacleHandler.Remove(obstacle);
                        _parent.InputHandler.ClearSelection();
                    }
                }

                if (ImGui.BeginPopup("ObstacleCreate"))
                {
                    foreach (var shape in _parent.ObstacleHandler.Shapes)
                    {
                        if (ImGui.Selectable(shape))
                        {
                            var shapeValue = (ObstacleShape)Enum.Parse(typeof(ObstacleShape), shape);
                            _parent.ObstacleHandler.AddDefault(shapeValue);

                            ImGui.CloseCurrentPopup();
                        }
                    }

                    ImGui.EndPopup();
                }

                ImGui.Separator();

                ImGui.PushStyleVar(ImGuiStyleVar.ChildRounding, 5);
                if (ImGui.BeginChild("ObstacleList", ImGui.GetContentRegionAvail(), true))
                {
                    if (_parent.ObstacleHandler.Obstacles.Count > 0)
                    {
                        for (int i = 0; i < _parent.ObstacleHandler.Obstacles.Count; i++)
                        {
                            var obstacle = _parent.ObstacleHandler.Obstacles[i];
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
                }

                ImGui.End();
            }
        }
        #endregion

        #region OPTIONS_WINDOW
        //private void RenderOptionsWindow()
        //{
        //    // options and info window
        //    if (ImGui.Begin("Options & Info",
        //        ImGuiWindowFlags.NoCollapse |
        //        ImGuiWindowFlags.NoMove |
        //        ImGuiWindowFlags.NoResize))
        //    {
        //        ImGui.SetWindowPos(new System.Numerics.Vector2(0, (int)(0.75 * (Window.Height - 19) + 19)));
        //        ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.25 * Window.Width - 2), (int)(0.25 * (Window.Height - 19))));

        //        string targetMode;
        //        switch (MainWindow.Mode)
        //        {
        //            case InteractionMode.Animate:
        //            case InteractionMode.ToDesign:
        //                targetMode = "Design";
        //                break;
        //            case InteractionMode.Design:
        //            case InteractionMode.ToAnimate:
        //                targetMode = "Animate";
        //                break;
        //            default:
        //                throw new ArgumentException("The given mode is unsupported!", "MainWindow.Mode");
        //        }

        //        if (ImGui.Button(targetMode))
        //        {
        //            MainWindow.SwitchMode();
        //        }
        //        if (ImGui.IsItemHovered())
        //        {
        //            ImGui.SameLine();
        //            ImGui.TextWrapped("Switch between design/animate modes");
        //        }

        //        //if (ImGui.Button("Screenshot"))
        //        //{
        //        //    // inform the input handler that the window capture has been queried
        //        //    InputHandler.Capture = true;
        //        //}
        //        //if (ImGui.IsItemHovered())
        //        //{
        //        //    ImGui.SameLine();
        //        //    ImGui.TextWrapped("Takes a picture of the entire window");
        //        //}

        //        //// savepath for captured screenshot
        //        //ImGui.InputText("Savepath", ref InputHandler.ScreenshotsPath, 100);
        //        //InputHandler.TextIsEdited = ImGui.IsItemActive();

        //        // application current framerate
        //        ImGui.SetCursorScreenPos(new System.Numerics.Vector2(8, Window.Height - 8 - ImGui.CalcTextSize("Framerate:").Y));
        //        ImGui.Text(string.Format("Framerate: {0:F1} FPS", ImGui.GetIO().Framerate));

        //        ImGui.End();
        //    }
        //}
        #endregion

        #region PROPERTIES_WINDOW
        private void RenderPropertiesWindow()
        {
            if (_parent.InputHandler.SelectedObject is Manipulator manipulator)
            {
                RenderPropertiesWindowTemplate("Manipulator properties", manipulator, ManipulatorProperties);
            }
            else if (_parent.InputHandler.SelectedObject is Joint joint)
            {
                RenderPropertiesWindowTemplate("Joint properties", joint, JointProperties);
            }
            else if (_parent.InputHandler.SelectedObject is Link link)
            {
                RenderPropertiesWindowTemplate("Link properties", link, LinkProperties);
            }
            else if (_parent.InputHandler.SelectedObject is Obstacle obstacle)
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
                ImGui.SetWindowPos(new System.Numerics.Vector2((int)(0.25 * Window.Size.X), (int)(0.7 * Window.Size.Y)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.3 * Window.Size.X - 2), (int)(0.3 * Window.Size.Y)));

                // perform the necessary actions
                renderProperties(selectable);

                // remove window's ID from the stack, just in case
                ImGui.PopID();
            }
        }

        private void ManipulatorProperties(Manipulator manipulator)
        {
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
                            case InverseKinematicsSolverType.HillClimbing:
                                manipulator.Controller.InverseKinematicsSolver = HillClimbing.Default();
                                break;
                        }
                    }

                    ImGui.Separator();

                    // inverse kinematics solver properties
                    ImGui.InputFloat("Threshold", ref manipulator.Controller.InverseKinematicsSolver.Threshold);
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
                    else if (manipulator.Controller.InverseKinematicsSolver is HillClimbing hillClimbing)
                    {
                        ImGui.InputFloat("Max step size", ref hillClimbing.MaxStepSize);
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
                    ImGui.InputFloat("Threshold", ref manipulator.Controller.PathPlanner.Threshold);

                    if (manipulator.Controller.PathPlanner is RRT rrt)
                    {
                        ImGui.Checkbox($"Show tree", ref rrt.ShowTree);
                        ImGui.InputFloat("Step", ref rrt.Step);
                        ImGui.Checkbox("Enable trimming", ref rrt.EnableTrimming);
                        ImGui.InputInt("Trim period", ref rrt.TrimPeriod);

                        if (rrt is ARRT arrt)
                        {
                            ImGui.InputInt("Attractors count", ref arrt.AttractorsCount);
                        }
                        else
                        {
                            ImGui.InputInt("Goal bias period", ref rrt.GoalBiasPeriod);
                        }
                    }
                    else if (manipulator.Controller.PathPlanner is GeneticAlgorithm geneticAlgorithm)
                    {
                        // TODO: add genetic algorithm related properties
                        ImGui.InputInt("Offspring size", ref geneticAlgorithm.OffspringSize);
                        ImGui.InputInt("Survival size", ref geneticAlgorithm.SurvivalSize);
                        ImGui.InputInt("Bezier control points", ref geneticAlgorithm.BezierControlPointsCount);
                        ImGui.InputFloat("Bezier step", ref geneticAlgorithm.BezierStep);
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
            //ImGui.Checkbox("Activate", ref joint.Active);  // TODO: for debug use only

            ImGui.Checkbox("Show collider", ref joint.ShowCollider);
            var axis = joint.InitialAxis.ToNumerics3();
            ImGui.InputFloat3("Axis", ref axis);
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
            ImGui.Checkbox("Show collider", ref link.ShowCollider);

            // TODO: add length property
            if (link.Collider is CylinderCollider cylinder)
            {
                ImGui.InputFloat("Radius", ref cylinder.Radius);
                ImGui.InputFloat("Half length", ref cylinder.HalfLength, 0, 0, null, ImGuiInputTextFlags.ReadOnly);  // TODO: this should not be read-only; implement!
            }

            //if (ImGui.BeginTabBar("LinkTabs"))
            //{
            //    if (ImGui.BeginTabItem("Model"))
            //    {
            //        // TODO: add model related properties

            //        ImGui.EndTabItem();
            //    }

            //    if (ImGui.BeginTabItem("Collider"))
            //    {
            //        ImGui.Checkbox("Show collider", ref link.ShowCollider);

            //        // TODO: add length property
            //        if (link.Collider is CylinderCollider cylinder)
            //        {
            //            ImGui.InputFloat("Radius", ref cylinder.Radius);
            //            ImGui.InputFloat("Half length", ref cylinder.HalfLength, 0, 0, null, ImGuiInputTextFlags.ReadOnly);  // TODO: this should not be read-only; implement!
            //        }

            //        ImGui.EndTabItem();
            //    }

            //    ImGui.EndTabBar();
            //}
        }

        private void ObstacleProperties(Obstacle obstacle)
        {
            ImGui.Checkbox("Show collider", ref obstacle.ShowCollider);
            ImGui.InputFloat3("Orientation", ref obstacle.Orientation);
            ImGui.InputFloat3("Position", ref obstacle.InitialPosition);

            ImGui.Separator();

            if (ImGui.BeginTabBar("ObstacleTabs"))
            {
                if (ImGui.BeginTabItem("Shape"))
                {
                    ImGui.Text($"Shape type: {obstacle.Shape}");

                    ImGui.Separator();

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
                    else if (obstacle.Collider is ConeCollider cone)
                    {
                        ImGui.InputFloat("Radius", ref cone.Radius);
                        ImGui.InputFloat("Height", ref cone.Height);
                    }

                    ImGui.EndTabItem();
                }

                if (ImGui.BeginTabItem("Physics"))
                {
                    int type = (int)obstacle.Type;
                    ImGui.Combo("Type", ref type,
                        _parent.PhysicsHandler.RigidBodyTypes,
                        _parent.PhysicsHandler.RigidBodyTypes.Length);
                    //obstacle.Type = (RigidBodyType)type;

                    if (obstacle.Type == RigidBodyType.Dynamic)
                    {
                        ImGui.InputFloat("Mass", ref obstacle.Mass);
                    }
                    else
                    {
                        ImGui.PushStyleVar(ImGuiStyleVar.Alpha, 0.5f);
                        ImGui.InputFloat("Mass", ref obstacle.Mass, 0, 0, null, ImGuiInputTextFlags.ReadOnly);
                        ImGui.PopStyleVar();
                    }

                    ImGui.EndTabItem();
                }

                if (ImGui.BeginTabItem("Path"))
                {
                    if (obstacle.Type == RigidBodyType.Kinematic)
                    {
                        var quarterWidth = 0.25f * ImGui.GetWindowWidth();

                        Path.Node current = obstacle.Path.First.Child;
                        int selectedIndex = -1;
                        ImGui.PushStyleVar(ImGuiStyleVar.ChildRounding, 5);
                        if (ImGui.BeginChild("ObstaclePath", new System.Numerics.Vector2(quarterWidth, ImGui.GetContentRegionAvail().Y), true,
                            ImGuiWindowFlags.HorizontalScrollbar))
                        {
                            while (current != null)
                            {
                                if (ImGui.Selectable($"Point {++selectedIndex}"))
                                {
                                    _selectedPoint = current;
                                    _selectedPathIndex = selectedIndex;
                                }

                                current = current.Child;
                            }

                            if (selectedIndex == -1)
                            {
                                ImGui.TextWrapped("Path is empty.");
                            }

                            ImGui.EndChild();
                        }

                        ImGui.SameLine();

                        ImGui.BeginGroup();

                        if (ImGui.Button("Add point"))
                        {
                            obstacle.Path.AddLast(new System.Numerics.Vector3[] 
                            { 
                                obstacle.Path.Last.Points[0] + System.Numerics.Vector3.UnitX 
                            }, null);
                        }

                        ImGui.Separator();

                        if (_selectedPoint != null)
                        {
                            ImGui.Text($"Point {_selectedPathIndex}");

                            var point = _selectedPoint.Points[0];
                            ImGui.InputFloat3("", ref point);
                            obstacle.Path.ChangeNode(_selectedPoint, new System.Numerics.Vector3[] { point }, null);
                        }

                        ImGui.EndGroup();
                    }
                    else
                    {
                        ImGui.TextWrapped("A path can be set only for Kinematic obstacles (see Physics tab).");
                    }

                    ImGui.EndTabItem();
                }

                ImGui.EndTabBar();
            }
        }

        private void SwapPropertiesWindows()
        {
            _swapPropertiesWindows = !_swapPropertiesWindows;
        }
        #endregion

        #region STATISTICS_WINDOW
        private void RenderStatisticsWindow()
        {
            if (ImGui.Begin("Statistics",
                    ImGuiWindowFlags.NoCollapse |
                    ImGuiWindowFlags.NoMove |
                    ImGuiWindowFlags.NoResize |
                    ImGuiWindowFlags.HorizontalScrollbar))
            {
                // set position and size of the window
                ImGui.SetWindowPos(new System.Numerics.Vector2((int)(0.25 * Window.Size.X), (int)(0.7 * Window.Size.Y)));
                ImGui.SetWindowSize(new System.Numerics.Vector2((int)(0.3 * Window.Size.X - 2), (int)(0.3 * Window.Size.Y)));

                if (_parent.ManipulatorHandler.Count > 0)
                {
                    var quarterWidth = 0.25f * ImGui.GetWindowWidth();

                    int selectedIndex = -1;
                    ImGui.PushStyleVar(ImGuiStyleVar.ChildRounding, 5);
                    if (ImGui.BeginChild("ManipulatorStat", new System.Numerics.Vector2(quarterWidth, ImGui.GetContentRegionAvail().Y), true,
                        ImGuiWindowFlags.HorizontalScrollbar))
                    {
                        foreach (var manipulator in _parent.ManipulatorHandler.Manipulators)
                        {
                            if (ImGui.Selectable($"Manip {++selectedIndex}"))
                            {
                                _selectedStatIndex = selectedIndex;
                            }
                        }

                        ImGui.EndChild();
                    }

                    ImGui.SameLine();

                    ImGui.BeginGroup();

                    if (_selectedStatIndex != -1)
                    {
                        ImGui.Text($"Manipulator {_selectedStatIndex}");

                        ImGui.Separator();

                        RenderAlgorithmStatistics(_parent.ManipulatorHandler.Manipulators[_selectedStatIndex]);
                    }

                    ImGui.EndGroup();
                }
                else
                {
                    ImGui.Text("No manipulators on the scene.");
                }
            }
        }

        private void RenderAlgorithmStatistics(Manipulator manipulator)
        {
            ImGui.Text($"Planning iterations: {manipulator.Controller.PathPlanner.Iterations}");
            ImGui.Text($"Planning time: {manipulator.Controller.PathPlanner.Timer.Elapsed.TotalSeconds : 0.000} s");
            ImGui.Text($"Control time: {manipulator.Controller.MotionController.Timer.Elapsed.TotalSeconds : 0.000} s");
            
            ImGui.Separator();

            if (manipulator.Controller.PathPlanner is RRT rrt)
            {
                ImGui.Text($"Tree size: {(rrt.Tree == null ? 0 : rrt.Tree.Count)} vertices");
            }
            else if (manipulator.Controller.PathPlanner is GeneticAlgorithm geneticAlgorithm)
            {
                ImGui.Text($"Dominant weight: {(geneticAlgorithm.Dominant == null ? float.PositiveInfinity : geneticAlgorithm.Dominant.Weight) : 0.000}");
            }
        }
        #endregion

        #region SELECTION
        public void OnSelectedObjectChanged(ObjectSelectEventArgs e)
        {
            SwapPropertiesWindows();
        }

        // TODO: perhaps should be moved to InputHandler somehow?
        private void UpdateSelection(object selected)
        {
            /*_parent.InputHandler.ClearSelection();

            if (selected == _parent.InputHandler.SelectedObject)
            {
                //_parent.InputHandler.SelectedObject = null;
            }
            else
            {
                if (selected is Manipulator manipulator)
                {
                    _parent.InputHandler.AddSelection(manipulator.Joints.Select(joint => joint.Collider.Body));
                    _parent.InputHandler.AddSelection(manipulator.Links.Select(link => link.Collider.Body));
                }
                else if (selected is Joint joint)
                {
                    _parent.InputHandler.AddSelection(joint.Collider.Body);
                }
                else if (selected is Link link)
                {
                    _parent.InputHandler.AddSelection(link.Collider.Body);
                }
                else if (selected is Obstacle obstacle)
                {
                    _parent.InputHandler.AddSelection(obstacle.Collider.Body);
                }

                _parent.InputHandler.SelectedObject = selected;
                SwapPropertiesWindows();
            }*/
        }

        private ImGuiTreeNodeFlags GetTreeNodeSelectionFlag(object selected)
        {
            return selected == _parent.InputHandler.SelectedObject ? ImGuiTreeNodeFlags.Selected : ImGuiTreeNodeFlags.None;
        }
        #endregion
    }
}
