using System;
using System.IO;
using System.Reflection;
using System.Linq;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;

using OpenTK.Windowing.GraphicsLibraryFramework;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;

using BulletSharp;
using Physics;
using ImGuiNET;
using Logic;
using Assimp;

namespace Graphics
{
    public enum InteractionMode
    {
        Design,
        Simulate
    }

    public readonly struct CameraMoveEventArgs
    {
        public Vector2i Direction { get; init; }
        public float Time { get; init; }
    }

    public readonly struct CameraRotateEventArgs
    {
        public Vector2 Delta { get; init; }
    }

    public readonly struct CameraZoomEventArgs
    {
        public float Delta { get; init; }
        public float AspectRatio { get; init; }
    }

    public readonly struct ObjectSelectEventArgs
    {
        public ISelectable PreviousObject { get; init; }
        public ISelectable Object { get; init; }
    }

    public readonly struct InteractionModeSwitchEventArgs
    {
        public InteractionMode PreviousMode { get; init; }
        public InteractionMode Mode { get; init; }
    }

    public class InputHandler
    {
        private readonly MainWindow _parent;
        private readonly TranslationalWidget _translationalWidget;
        private readonly HashSet<CollisionObject> _selectedObjects = new();

        // assembly related data
        public static string ProjectName { get; } = Assembly.GetEntryAssembly().GetName().Name;
        public static string ExeDirectory { get; } = Environment.CurrentDirectory;

        // resources paths
        public string LinkPath { get; } = $"{ExeDirectory}/Resources/Models/manipulator/Link.obj";
        public string JointPath { get; } = $"{ExeDirectory}/Resources/Models/manipulator/Joint.obj";
        public string GripperPath { get; } = $"{ExeDirectory}/Resources/Models/manipulator/Gripper.obj";
        public string ScreenshotsPath { get; } = $"{ExeDirectory}/Screenshots";

        // shaders paths
        public string VertexShader { get; } = $"{ExeDirectory}/Resources/Shaders/VertexShader.glsl";
        public string GenericFragmentShader { get; } = $"{ExeDirectory}/Resources/Shaders/LineShader.glsl";
        public string ComplexFragmentShader { get; } = $"{ExeDirectory}/Resources/Shaders/FragmentShader.glsl";

        public InteractionMode InteractionMode { get; private set; } = InteractionMode.Design;
        public object SelectedObject => _selectedObjects.Count == 1 ? _selectedObjects.First().UserObject : null;

        // events
        public event Action<ObjectSelectEventArgs> SelectedObjectChanged;
        public event Action<CameraMoveEventArgs> CameraPositionChanged;
        public event Action<CameraRotateEventArgs> CameraOrientationChanged;
        public event Action<CameraZoomEventArgs> CameraZoomChanged;
        public event Action<InteractionModeSwitchEventArgs> InteractionModeSwitched;

        public InputHandler(MainWindow parent)
        {
            _parent = parent;

            // widgets
            _translationalWidget = new TranslationalWidget(Vector3.Zero);

            // subscribe widgets to the events
            SelectedObjectChanged += _translationalWidget.OnSelectedObjectChanged;
        }

        public void Update(FrameEventArgs e)
        {
            if (InteractionMode == InteractionMode.Design && !ImGui.IsWindowHovered(
                ImGuiHoveredFlags.AnyWindow |
                ImGuiHoveredFlags.AllowWhenBlockedByPopup))  // TODO: perhaps use some other way of obtaining current mode?
            {
                PollInteraction();
            }

            PollMouse();

            PollKeyboard(e);
        }

        public void OnInteractionModeSwitched(InteractionModeSwitchEventArgs e)
        {
            foreach (var selected in _selectedObjects)
            {
                var model = (selected.UserObject as ISelectable).Model;
                switch (e.Mode)
                {
                    case InteractionMode.Design:
                        // restore Selected flag for selected objects
                        model.RenderFlags |= RenderFlags.Selected;
                        break;
                    case InteractionMode.Simulate:
                        // clear Selected flag for selected objects
                        model.RenderFlags &= ~RenderFlags.Selected;
                        break;
                }
            }
        }

        public void Render(Shader shader, Action action)
        {
            _translationalWidget.Render(shader, action);
        }

        private void PollMouse()
        {
            // apply rotation on MMB only if the mouse moved
            if (_parent.MouseState.IsButtonDown(MouseButton.Middle) && 
                _parent.MouseState.Delta != Vector2.Zero)
            {
                CameraOrientationChanged?.Invoke(new CameraRotateEventArgs
                {
                    Delta = _parent.MouseState.Delta
                });
            }

            // apply zoom on mouse wheel only if no GUI window is currently hovered over
            if (_parent.MouseState.ScrollDelta.Y != 0 &&
                !ImGui.IsWindowHovered(ImGuiHoveredFlags.AnyWindow))
            {
                CameraZoomChanged?.Invoke(new CameraZoomEventArgs
                {
                    Delta = _parent.MouseState.ScrollDelta.Y,
                    AspectRatio = _parent.ViewportAspectRatio
                });
            }
        }

        private void PollInteraction()
        {
            var ray = Ray.Cast(_parent.Camera, MouseToViewportNDC());

            PollSelection(ray);

            PollWidgets(ray);
        }

        private void PollSelection(Ray ray)  // TODO: raycast is performed wrong on shapes' edges; check!
        {
            if (!_parent.MouseState.IsButtonPressed(MouseButton.Right))
                return;

            var selectedPrev = SelectedObject;

            var target = _parent.PhysicsHandler.RayTest(ray);
            if (target != null)
            {
                if (_parent.KeyboardState.IsKeyDown(Keys.LeftControl))
                {
                    if (_selectedObjects.Contains(target))
                    {
                        RemoveSelection(target);
                    }
                    else
                    {
                        AddSelection(target);
                    }
                }
                else
                {
                    ClearSelection();
                    AddSelection(target);
                }
            }
            else
            {
                ClearSelection();
            }

            var selectedCurr = SelectedObject;
            if (selectedCurr != selectedPrev)
            {
                SelectedObjectChanged?.Invoke(new ObjectSelectEventArgs
                {
                    PreviousObject = selectedPrev as ISelectable,
                    Object = selectedCurr as ISelectable
                });
            }
        }

        private void PollWidgets(Ray ray)
        {
            if (SelectedObject is not ITranslatable)
                return;

            // scale the widget so that its size remains constant
            _translationalWidget.Scale(ray.CameraPosition);

            if (_parent.MouseState.IsButtonPressed(MouseButton.Left))
            {
                _translationalWidget.TryActivate(ray);
            }

            if (!_translationalWidget.IsActive)
                return;

            if (_parent.MouseState.IsButtonDown(MouseButton.Left))
            {
                _translationalWidget.Operate(ray);
            }
            else
            {
                _translationalWidget.Deactivate();
            }
        }

        public void AddSelection(IEnumerable<CollisionObject> collisionObjects)
        {
            foreach (var collisionObject in collisionObjects)
                AddSelection(collisionObject);
        }

        private void AddSelection(CollisionObject collisionObject)
        {
            if (collisionObject.UserObject is ISelectable selectable)
            {
                _selectedObjects.Add(collisionObject);
                selectable.Model.RenderFlags |= RenderFlags.Selected;
            }
        }

        public void RemoveSelection(CollisionObject collisionObject)
        {
            (collisionObject.UserObject as ISelectable).Model.RenderFlags &= ~RenderFlags.Selected;
            _selectedObjects.Remove(collisionObject);
        }

        public void ClearSelection()
        {
            foreach (var collisionObject in _selectedObjects)
                (collisionObject.UserObject as ISelectable).Model.RenderFlags &= ~RenderFlags.Selected;
            _selectedObjects.Clear();
        }

        private void PollKeyboard(FrameEventArgs e)
        {
            if (_parent.KeyboardState.IsKeyDown(Keys.Escape))
                _parent.Close();

            // TODO: this block should only be executed if text is not edited;
            // the corresponding identifier should be placed in GuiHandler class
            int dx = 0, dy = 0;
            if (_parent.KeyboardState.IsKeyDown(Keys.W)) dy += 1;
            if (_parent.KeyboardState.IsKeyDown(Keys.S)) dy -= 1;
            if (_parent.KeyboardState.IsKeyDown(Keys.A)) dx -= 1;
            if (_parent.KeyboardState.IsKeyDown(Keys.D)) dx += 1;

            if (dx != 0 || dy != 0)
            {
                CameraPositionChanged?.Invoke(new CameraMoveEventArgs
                {
                    Direction = new Vector2i { X = dx, Y = dy },
                    Time = (float)e.Time
                });
            }

            if (_parent.KeyboardState.IsKeyPressed(Keys.P))
            {
                InteractionModeSwitched?.Invoke(new InteractionModeSwitchEventArgs
                {
                    PreviousMode = InteractionMode,
                    Mode = InteractionMode = 1 - InteractionMode
                });
            }

            if (_parent.KeyboardState.IsKeyPressed(Keys.K))
                CaptureScreenFull();
        }

        private Vector2 MouseToViewportNDC()
        {
            // mouse position relative to the main viewport, normalized to [-1, 1] range
            var mousePosition = 2 * (_parent.MouseState.Position - _parent.ViewportOrigin) / _parent.ViewportSize - Vector2.One;

            // Y axis is oriented upwards in NDC
            return new(mousePosition.X, -mousePosition.Y);
        }

        public void CaptureScreenFull() => CaptureScreenArea(Vector2i.Zero, _parent.Size);

        public void CaptureScreenViewport() => CaptureScreenArea(_parent.ViewportOrigin, _parent.ViewportSize);

        // TODO: review
        private void CaptureScreenArea(Vector2i origin, Vector2i size)
        {
            // taking a picture of a viewport
            byte[,,] img = new byte[size.Y, size.X, 3];
            GL.ReadPixels(origin.X, origin.Y, size.X, size.Y, OpenTK.Graphics.OpenGL4.PixelFormat.Rgb, PixelType.UnsignedByte, img);

            Console.WriteLine("Capturing screen...");
            System.Threading.Tasks.Task.Run(() =>
            {
                using (var bitmap = new Bitmap(size.X, size.Y))
                {
                    // write all captured data to that bitmap
                    for (int i = 0; i < size.Y; i++)
                    {
                        for (int j = 0; j < size.X; j++)
                        {
                            bitmap.SetPixel(j, size.Y - 1 - i, Color.FromArgb(img[i, j, 0], img[i, j, 1], img[i, j, 2]));
                        }
                    }

                    // get the image format
                    string savepathLow = ScreenshotsPath.ToLower();
                    string formatString = "";
                    for (int i = savepathLow.Length - 1; i >= 0; i--)
                    {
                        formatString += savepathLow[i];

                        if (savepathLow[i] == '.' || savepathLow[i] == '/' || savepathLow[i] == '\\')
                            break;
                    }

                    // reverse string to obtain the format
                    var chars = formatString.ToCharArray();
                    Array.Reverse(chars);
                    formatString = new string(chars);

                    // check format support
                    ImageFormat format = ImageFormat.Bmp;
                    bool formatSpecified = true;
                    switch (formatString)
                    {
                        case ".bmp":
                            format = ImageFormat.Bmp;
                            break;
                        case ".jpg":
                        case ".jpeg":
                            format = ImageFormat.Jpeg;
                            break;
                        case ".png":
                            format = ImageFormat.Png;
                            break;
                        case ".gif":
                            format = ImageFormat.Gif;
                            break;
                        default:
                            formatSpecified = false;
                            break;
                    }

                    // save the obtained bitmap
                    if (formatSpecified)
                        bitmap.Save(ScreenshotsPath, format);
                    else
                        bitmap.Save(ScreenshotsPath + ".bmp", ImageFormat.Bmp);
                }
            });
        }

        public void Dispose()
        {
            // dispose of the widgets
            _translationalWidget.Dispose();
        }
    }
}
