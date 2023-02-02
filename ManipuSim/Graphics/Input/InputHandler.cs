﻿using System;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;

using OpenTK.Windowing.GraphicsLibraryFramework;
using OpenTK.Windowing.Desktop;
using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;

using BulletSharp;
using Physics;
using System.Collections.Generic;
using ImGuiNET;
using Logic;
using OpenTK.Windowing.Common;

namespace Graphics
{
    public static class InputHandler
    {
        public static string ExeDirectory => Environment.CurrentDirectory;

        public static string NanosuitPath => ExeDirectory + "/Resources/Models/nanosuit/nanosuit.obj";
        public static string LinkPath => ExeDirectory + "/Resources/Models/manipulator/Link.obj";
        public static string JointPath => ExeDirectory + "/Resources/Models/manipulator/Joint.obj";
        public static string GripperPath => ExeDirectory + "/Resources/Models/manipulator/Gripper.obj";

        private static string _screenshotsPath = ExeDirectory + "/Screenshot";
        public static ref string ScreenshotsPath => ref _screenshotsPath;

        // variables for mouse state processing
        private static bool _firstMove;
        private static MouseState _lastState;

        public static Vector2 CursorPositionNDC { get; private set; }
        public static Ray RaycastResult { get; private set; }
        public static bool TextIsEdited { get; set; }
        public static bool Capture { get; set; }

        public static TranslationalWidget TranslationalWidget { get; set; }

        public static List<CollisionObject> SelectedObjects { get; } = new List<CollisionObject>();
        public static object CurrentSelectedObject { get; set; }
        public static event EventHandler SelectedObjectChanged;

        public static void ToAnimate()
        {
            // clear Selected flag for selected objects
            foreach (var select in SelectedObjects)
            {
                (select.UserObject as ISelectable).Model.RenderFlags &= ~RenderFlags.Selected;
            }
        }

        public static void ToDesign()
        {
            // restore Selected flag for selected objects
            foreach (var select in SelectedObjects)
            {
                (select.UserObject as ISelectable).Model.RenderFlags |= RenderFlags.Selected;
            }
        }

        public static void PollEvents(GameWindow window, Camera camera, MouseState mouseState, KeyboardState keyboardState, FrameEventArgs e)
        {
            // update cursor position in NDC space
            CursorPositionNDC = MouseToNDC(window, mouseState);  // TODO: consider creating an extension method CursorPositionNDC() for MouseState

            // perform a raycast and store the result for later use
            RaycastResult = Ray.Cast(ref camera.ViewMatrix, ref camera.ProjectionMatrix);

            // attach widgets to or detach from the selected objects
            AttachWidgets(camera);

            // scale all axes so that their size on screen remains fixed
            TranslationalWidget.Scale(camera);

            if (MainWindow.Mode == InteractionMode.Design && !ImGui.IsWindowHovered(
                ImGuiHoveredFlags.AnyWindow | 
                ImGuiHoveredFlags.AllowWhenBlockedByPopup))  // TODO: perhaps use some other way of obtaining current mode?
            {
                // check whether any physical object is being selected
                PollSelection(window, mouseState, keyboardState);

                // poll widget for interaction
                TranslationalWidget.Poll(camera, RaycastResult, mouseState, _lastState);
            }

            // poll the mouse for events
            PollMouse(window, camera, mouseState);

            // poll the keyboard for events
            PollKeyboard(window, camera, keyboardState, e);

            // poll the screen for events
            PollScreen(window, keyboardState);
        }

        private static void PollMouse(GameWindow window, Camera camera, MouseState mouseState)
        {
            if (_firstMove) // this bool variable is initially set to true
            {
                _firstMove = false;
            }
            else if (mouseState.IsButtonDown(MouseButton.Middle))  // update camera orientation if the middle button is pressed
            {
                // Apply the camera pitch and yaw (we clamp the pitch in the camera class)
                // TODO: LastMouseState returns incorrect states; report!
                var delta = window.MouseState.Position - _lastState.Position;
                camera.Yaw += delta.X * camera.Sensitivity;
                camera.Pitch -= delta.Y * camera.Sensitivity; // reversed since y-coordinates range from bottom to top

                Console.WriteLine($"{window.MouseState.Position}, {window.MousePosition}, {window.MouseState.PreviousPosition}, {window.MouseState.Delta}");
            }

            // update last mouse state after all necessary queries
            _lastState = window.MouseState;
        }

        private static Vector2 MouseToNDC(GameWindow window, MouseState mouseState)
        {
            // cursor position relative to the window
            //var cursorWindow = window.PointToClient(new Vector2i((int)mouseState.X, (int)mouseState.Y));

            // cursor position relative to the main viewport
            var cursorViewport = new Vector2(mouseState.X - 0.25f * window.Size.X, mouseState.Y);
            //var cursorViewport = new Vector2(cursorWindow.X - 0.25f * window.Size.X, cursorWindow.Y);

            // return cursor position in NDC coordinates
            return new Vector2(
                (cursorViewport.X / (0.75f * window.Size.X) - 0.5f) * 2,
                (0.5f - cursorViewport.Y / window.Size.Y) * 2);
        }

        private static void PollSelection(GameWindow window, MouseState mouseState, KeyboardState keyboardState)  // TODO: raycast is performed wrong on shapes' edges; check!
        {
            var startWorld = RaycastResult.StartWorld.ToBullet3();
            var endWorld = RaycastResult.EndWorld.ToBullet3();

            using (var raycastCallback = new ClosestRayResultCallback(ref startWorld, ref endWorld))
            {
                PhysicsHandler.RayTestRef(ref startWorld, ref endWorld, raycastCallback);
                if (/*window.IsMouseButtonPressed(MouseButton.Right)*/mouseState.IsButtonPressed(MouseButton.Right))
                {
                    if (raycastCallback.HasHit)
                    {
                        if (!keyboardState.IsKeyDown(Keys.LeftControl) && SelectedObjects.Find(x => x != raycastCallback.CollisionObject) != null)
                        {
                            // Control is not pressed and other objects are already selected ---> clear selection and add the new object to the selection
                            ClearAndAddSelection(raycastCallback.CollisionObject);
                        }
                        else
                        {
                            // add the object to the selection if it's not there yet
                            if (!SelectedObjects.Contains(raycastCallback.CollisionObject))
                            {
                                AddSelection(raycastCallback.CollisionObject);
                            }
                            else
                            {
                                RemoveSelection(raycastCallback.CollisionObject);
                            }
                        }
                    }
                    else
                    {
                        // no object is selected ---> clear selection
                        ClearSelection();
                    }
                }
            }
        }

        public static void AttachWidgets(Camera camera)
        {
            if (SelectedObjects.Count == 1)
            {
                var selected = SelectedObjects[0].UserObject as ISelectable;
                if (selected != CurrentSelectedObject)
                {
                    CurrentSelectedObject = selected;

                    // fire an event of selected object being changed
                    SelectedObjectChanged?.Invoke(selected, EventArgs.Empty);
                }

                // attach the widget
                TranslationalWidget.Attach(CurrentSelectedObject as ITranslatable, camera);
            }
            else
            {
                if (SelectedObjects.Count == 0 || !(CurrentSelectedObject is Manipulator))
                    CurrentSelectedObject = null;

                // detach the widget
                TranslationalWidget.Detach();
            }
        }

        public static void AddSelection(CollisionObject collisionObject)
        {
            if (collisionObject.UserObject is ISelectable selectable)
            {
                SelectedObjects.Add(collisionObject);
                selectable.Model.RenderFlags |= RenderFlags.Selected;
            }
        }

        public static void AddSelection(IEnumerable<CollisionObject> collisionObjects)
        {
            foreach (var collisionObject in collisionObjects)
            {
                AddSelection(collisionObject);
            }
        }

        public static void RemoveSelection(CollisionObject collisionObject)
        {
            (collisionObject.UserObject as ISelectable).Model.RenderFlags &= ~RenderFlags.Selected;
            SelectedObjects.Remove(collisionObject);
        }

        public static void ClearSelection()
        {
            SelectedObjects.ForEach(x => (x.UserObject as ISelectable).Model.RenderFlags &= ~RenderFlags.Selected);
            SelectedObjects.Clear();
        }

        public static void ClearAndAddSelection(CollisionObject collisionObject)
        {
            ClearSelection();
            AddSelection(collisionObject);
        }

        private static void PollKeyboard(GameWindow window, Camera camera, KeyboardState keyboardState, FrameEventArgs e)
        {
            // exit program if queried
            if (keyboardState.IsKeyDown(Keys.Escape))
            {
                window.Close();
            }

            if (!TextIsEdited)
            {
                // panning
                if (keyboardState.IsKeyDown(Keys.W))
                    camera.Position += camera.Up * camera.Speed * (float)e.Time; // Up 
                if (keyboardState.IsKeyDown(Keys.S))
                    camera.Position -= camera.Up * camera.Speed * (float)e.Time; // Down
                if (keyboardState.IsKeyDown(Keys.A))
                    camera.Position -= camera.Right * camera.Speed * (float)e.Time; // Left
                if (keyboardState.IsKeyDown(Keys.D))
                    camera.Position += camera.Right * camera.Speed * (float)e.Time; // Right
            }
        }

        private static void PollScreen(GameWindow window, KeyboardState keyboardState)
        {
            if (window.IsKeyPressed(Keys.K))
                CaptureScreenFull(window);
        }

        public static void CaptureScreenFull(GameWindow window)
        {
            CaptureScreenArea(0, 0, window.Size.X, window.Size.Y);
        }

        public static void CaptureScreenWorkspace(GameWindow window)
        {
            CaptureScreenArea((int)(0.25 * window.Size.X), 0, (int)(0.75 * window.Size.X), window.Size.Y);
        }

        private static void CaptureScreenArea(int x, int y, int width, int height)
        {
            // taking a picture of a viewport
            byte[,,] img = new byte[height, width, 3];
            GL.ReadPixels(x, y, width, height, OpenTK.Graphics.OpenGL4.PixelFormat.Rgb, PixelType.UnsignedByte, img);

            Console.WriteLine("Capturing screen...");
            System.Threading.Tasks.Task.Run(() =>
            {
                using (var bitmap = new Bitmap(width, height))
                {
                    // write all captured data to that bitmap
                    for (int i = 0; i < height; i++)
                    {
                        for (int j = 0; j < width; j++)
                        {
                            bitmap.SetPixel(j, height - 1 - i, Color.FromArgb(img[i, j, 0], img[i, j, 1], img[i, j, 2]));
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

        public static void Dispose()
        {
            // dispose of the widgets
            TranslationalWidget.Dispose();
        }
    }
}
