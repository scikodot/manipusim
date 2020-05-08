using System;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;

using OpenTK;
using OpenTK.Input;
using OpenTK.Graphics.OpenGL4;
using Assimp;
using BulletSharp;
using Physics;
using System.Collections.Generic;
using ImGuiNET;

namespace Graphics
{
    public static class InputHandler
    {
        private static readonly DirectoryInfo projDir = Directory.GetParent(Environment.CurrentDirectory).Parent;
        private static readonly DirectoryInfo solDir = projDir.Parent;

        public static string ProjectDirectory => projDir.FullName;
        public static string SolutionDirectory => solDir.FullName;

        public static string NanosuitPath => SolutionDirectory + @"\Resources\Models\nanosuit\nanosuit.obj";
        public static string LinkPath => SolutionDirectory + @"\Resources\Models\manipulator\Link.obj";
        public static string JointPath => SolutionDirectory + @"\Resources\Models\manipulator\Joint.obj";
        public static string GripperPath => SolutionDirectory + @"\Resources\Models\manipulator\Gripper.obj";

        private static string _screenshotsPath = SolutionDirectory + @"\Screenshots";
        public static ref string ScreenshotsPath => ref _screenshotsPath;

        // variables for mouse state processing
        private static bool _firstMove;
        private static Vector2 _lastPos;
        private static MouseState _lastState;

        public static Vector2 CursorPositionNDC { get; private set; }
        public static Ray RaycastResult { get; private set; }
        public static bool TextIsEdited { get; set; }

        public static AxesWidget Widget { get; set; }

        public static List<CollisionObject> SelectedObjects { get; } = new List<CollisionObject>();

        public static void PollEvents(GameWindow window, Camera camera, MouseState mouseState, KeyboardState keyboardState, FrameEventArgs e)
        {
            // update cursor position in NDC space
            CursorPositionNDC = MouseToNDC(window, mouseState);  // TODO: consider creating an extension method CursorPositionNDC() for MouseState

            // perform a raycast and store the result for later use
            RaycastResult = Ray.Cast(ref camera.ViewMatrix, ref camera.ProjectionMatrix);

            // check whether any physical object is being selected
            if (!ImGui.IsWindowHovered(ImGuiHoveredFlags.AnyWindow | ImGuiHoveredFlags.AllowWhenBlockedByPopup))
                PollSelection(mouseState, keyboardState);

            // poll the mouse for events
            PollMouse(window, camera, mouseState);

            // poll the keyboard for events
            PollKeyboard(window, camera, keyboardState, e);

            // poll the screen for events
            PollScreen(window);
        }

        private static void PollMouse(GameWindow window, Camera camera, MouseState mouse)
        {
            if (_firstMove) // this bool variable is initially set to true
            {
                _firstMove = false;
            }
            else if (mouse.MiddleButton == ButtonState.Pressed)  // update camera orientation if the middle button is pressed
            {
                // Calculate the offset of the mouse position
                var deltaX = mouse.X - _lastState.X;
                var deltaY = mouse.Y - _lastState.Y;

                // Apply the camera pitch and yaw (we clamp the pitch in the camera class)
                camera.Yaw += deltaX * camera.Sensitivity;
                camera.Pitch -= deltaY * camera.Sensitivity; // reversed since y-coordinates range from bottom to top
            }

            // poll widget for interaction
            Widget.Poll(camera, mouse);

            // update last mouse state after all necessary queries
            _lastState = mouse;
        }

        private static Vector2 MouseToNDC(GameWindow window, MouseState mouse)
        {
            // cursor position relative to window
            var cursorWindow = new Point(mouse.X - window.X, mouse.Y - window.Y);

            // take into account window borders
            cursorWindow.X -= (int)(0.25 * window.Width + 8);  // 8 - indent for resizing feature
            cursorWindow.Y -= 38;  // 38 = 2 * 8 + 22, where 8 - resizing, 22 - main titlebar height

            // return cursor position in NDC coordinates
            return new Vector2(
                (cursorWindow.X / (0.75f * window.Width) - 0.5f) * 2,
                ((float)(window.Height - cursorWindow.Y) / window.Height - 0.5f) * 2);
        }

        private static void PollSelection(MouseState mouseState, KeyboardState keyboardState)  // TODO: raycast is performed wrong on shapes' edges; check!
        {
            var startWorld = RaycastResult.StartWorld.ToBullet3();
            var endWorld = RaycastResult.EndWorld.ToBullet3();

            using (var raycastCallback = new ClosestRayResultCallback(ref startWorld, ref endWorld))
            {
                PhysicsHandler.World.RayTestRef(ref startWorld, ref endWorld, raycastCallback);
                if (mouseState.LeftButton == ButtonState.Pressed && _lastState.LeftButton == ButtonState.Released)
                {
                    if (raycastCallback.HasHit)
                    {
                        if (!keyboardState.IsKeyDown(Key.ControlLeft) && SelectedObjects.Find(x => x != raycastCallback.CollisionObject) != null)
                        {
                            // Control is not pressed and other objects are already selected ---> clear selection and add the new object to the selection
                            ClearSelection();
                            AddSelection(raycastCallback.CollisionObject);
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

            Console.SetCursorPosition(0, 10);
            Console.WriteLine($"Selected count: {SelectedObjects.Count}");
        }

        private static void AddSelection(CollisionObject collisionObject)
        {
            SelectedObjects.Add(collisionObject);
            (collisionObject.UserObject as ISelectable).Model.RenderFlags |= RenderFlags.Selected;
        }

        private static void RemoveSelection(CollisionObject collisionObject)
        {
            (collisionObject.UserObject as ISelectable).Model.RenderFlags &= ~RenderFlags.Selected;
            SelectedObjects.Remove(collisionObject);
        }

        private static void ClearSelection()
        {
            SelectedObjects.ForEach(x => (x.UserObject as ISelectable).Model.RenderFlags &= ~RenderFlags.Selected);
            SelectedObjects.Clear();
        }

        private static void PollKeyboard(GameWindow window, Camera camera, KeyboardState keyboard, FrameEventArgs e)
        {
            // exit program if queried
            if (keyboard.IsKeyDown(Key.Escape))
            {
                window.Exit();
            }

            if (!TextIsEdited)
            {
                // panning
                if (keyboard.IsKeyDown(Key.W))
                    camera.Position += camera.Up * camera.Speed * (float)e.Time; // Up 
                if (keyboard.IsKeyDown(Key.S))
                    camera.Position -= camera.Up * camera.Speed * (float)e.Time; // Down
                if (keyboard.IsKeyDown(Key.A))
                    camera.Position -= camera.Right * camera.Speed * (float)e.Time; // Left
                if (keyboard.IsKeyDown(Key.D))
                    camera.Position += camera.Right * camera.Speed * (float)e.Time; // Right
            }
        }

        public static bool Capture { get; set; }
        private static void PollScreen(GameWindow window)
        {
            if (Capture)  // TODO: try to implement an event-based system
            {
                CaptureScreenshot(window);
                Capture = false;
            }
        }

        public static void CaptureScreenshot(GameWindow window)
        {
            // taking a picture of a viewport
            byte[,,] img = new byte[window.Height, window.Width, 3];
            GL.ReadPixels(0, 0, window.Width, window.Height, OpenTK.Graphics.OpenGL4.PixelFormat.Rgb, PixelType.UnsignedByte, img);

            System.Threading.Tasks.Task.Run(() =>
            {
                // create a bitmap representing captured screenshot
                var bitmap = new Bitmap(window.Width, window.Height);

                // write all captured data to that bitmap
                for (int i = 0; i < window.Height; i++)
                {
                    for (int j = 0; j < window.Width; j++)
                    {
                        bitmap.SetPixel(j, window.Height - 1 - i, Color.FromArgb(img[i, j, 0], img[i, j, 1], img[i, j, 2]));
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
            });
        }
    }
}
