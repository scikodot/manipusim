using System;
using System.IO;
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

    public class InputHandler
    {
        public static string ExeDirectory { get; } = Environment.CurrentDirectory;

        public string LinkPath { get; } = $"{ExeDirectory}/Resources/Models/manipulator/Link.obj";
        public string JointPath { get; } = $"{ExeDirectory}/Resources/Models/manipulator/Joint.obj";
        public string GripperPath { get; } = $"{ExeDirectory}/Resources/Models/manipulator/Gripper.obj";
        public string ScreenshotsPath { get; } = $"{ExeDirectory}/Screenshots";

        private readonly MainWindow _parent;

        public bool TextIsEdited { get; set; }

        private readonly TranslationalWidget _translationalWidget;

        private readonly HashSet<CollisionObject> _selectedObjects = new();
        public CollisionObject SelectedObject => _selectedObjects.Count == 1 ? _selectedObjects.First() : null;

        // events
        public static event Action<ObjectSelectEventArgs> SelectedObjectChanged;
        public static event Action<CameraMoveEventArgs> CameraPositionChanged;
        public static event Action<CameraRotateEventArgs> CameraOrientationChanged;
        public static event Action<CameraZoomEventArgs> CameraZoomChanged;

        public InputHandler(MainWindow parent)
        {
            _parent = parent;

            // widgets
            _translationalWidget = new TranslationalWidget(Vector3.Zero, new (Vector3, Vector4)[3]
            {
                (new Vector3(1, 0, 0), new Vector4(1, 0, 0, 1)),
                (new Vector3(0, 1, 0), new Vector4(0, 1, 0, 1)),
                (new Vector3(0, 0, 1), new Vector4(0, 0, 1, 1))
            });

            // subscribe widgets to the events
            SelectedObjectChanged += _translationalWidget.
        }

        public void ToAnimate()
        {
            // clear Selected flag for selected objects
            foreach (var select in _selectedObjects)
            {
                (select.UserObject as ISelectable).Model.RenderFlags &= ~RenderFlags.Selected;
            }
        }

        public void ToDesign()
        {
            // restore Selected flag for selected objects
            foreach (var select in _selectedObjects)
            {
                (select.UserObject as ISelectable).Model.RenderFlags |= RenderFlags.Selected;
            }
        }

        public void Poll(FrameEventArgs e)
        {
            // update cursor position in NDC space
            CursorPositionNDC = MouseToViewportNDC();  // TODO: consider creating an extension method CursorPositionNDC() for MouseState

            // attach widgets to or detach from the selected objects
            AttachWidgets(camera);

            // scale all axes so that their size on screen remains fixed
            TranslationalWidget.Scale(camera);

            if (MainWindow.Mode == InteractionMode.Design && !ImGui.IsWindowHovered(
                ImGuiHoveredFlags.AnyWindow | 
                ImGuiHoveredFlags.AllowWhenBlockedByPopup))  // TODO: perhaps use some other way of obtaining current mode?
            {
                PollInteraction();
            }

            PollMouse();

            PollKeyboard(e);
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

        // 
        private Vector2 MouseToViewportNDC()
        {
            // mouse position relative to the main viewport, normalized to [-1, 1] range
            var mousePosition = 2 * (_parent.MouseState.Position - _parent.ViewportOrigin) / _parent.ViewportSize - Vector2.One;
            
            // Y axis is oriented upwards in NDC
            return new(mousePosition.X, -mousePosition.Y);
        }

        private void PollInteraction()
        {
            var ray = Ray.Cast(_parent.Camera);

            PollSelection(ray);

            PollWidgets(ray);
        }

        private void PollSelection(Ray ray)  // TODO: raycast is performed wrong on shapes' edges; check!
        {
            var previousObject = SelectedObject;
            if (_parent.MouseState.IsButtonPressed(MouseButton.Right))
            {
                var startWorld = ray.StartWorld.ToBullet3();
                var endWorld = ray.EndWorld.ToBullet3();

                using var raycastCallback = new ClosestRayResultCallback(ref startWorld, ref endWorld);
                PhysicsHandler.RayTestRef(ref startWorld, ref endWorld, raycastCallback);
                if (raycastCallback.HasHit)
                {
                    var target = raycastCallback.CollisionObject;
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
            }

            var currentObject = SelectedObject;
            if (currentObject != previousObject)
            {
                SelectedObjectChanged?.Invoke(new ObjectSelectEventArgs
                {
                    PreviousObject = previousObject.UserObject as ISelectable,
                    Object = currentObject.UserObject as ISelectable
                });
            }
        }

        private void PollWidgets(Ray ray)
        {
            if (SelectedObject.UserObject is not ITranslatable translatable)
                return;

            if (_parent.MouseState.IsButtonPressed(MouseButton.Left))
            {
                // try activate widget
                _translationalWidget.TryActivate(ray);
            }

            if (!_translationalWidget.IsActive)
                return;

            if (_parent.MouseState.IsButtonDown(MouseButton.Left))
            {
                // operate widget
                _translationalWidget.Operate(ray);
            }
            else
            {
                // deactivate widget
            }

            _translationalWidget.Poll(ray);
        }

        public void AttachWidgets()
        {
            if (_selectedObjects.Count == 1)
            {
                var selected = _selectedObjects..UserObject as ISelectable;
                if (selected != SelectedObject)
                {
                    SelectedObject = selected;

                    // fire an event of selected object being changed
                    SelectedObjectChanged?.Invoke(new()
                    {

                    });
                }

                // attach the widget
                _translationalWidget.Attach(SelectedObject as ITranslatable, _parent.Camera);
            }
            else
            {
                if (_selectedObjects.Count == 0 || SelectedObject is not Manipulator)
                    SelectedObject = null;

                // detach the widget
                TranslationalWidget.Detach();
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

        private void ClearSelection()
        {
            foreach (var collisionObject in _selectedObjects)
                (collisionObject.UserObject as ISelectable).Model.RenderFlags &= ~RenderFlags.Selected;
            _selectedObjects.Clear();
        }

        private void PollKeyboard(FrameEventArgs e)
        {
            if (_parent.KeyboardState.IsKeyDown(Keys.Escape))
                _parent.Close();

            if (!TextIsEdited)
            {
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
            }

            if (_parent.KeyboardState.IsKeyPressed(Keys.K))
                CaptureScreenFull();
        }

        public void CaptureScreenFull() => CaptureScreenArea(Vector2i.Zero, _parent.Size);

        public void CaptureScreenViewport() => CaptureScreenArea(_parent.ViewportOrigin, _parent.ViewportSize);

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
