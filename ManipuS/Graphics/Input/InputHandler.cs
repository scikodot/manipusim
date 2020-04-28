using System;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;

using OpenTK;
using OpenTK.Input;
using OpenTK.Graphics.OpenGL4;

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

        public static AxesWidget Widget;

        public static bool TextIsEdited;

        public static void PollEvents(GameWindow window, Camera camera, MouseState mouse, KeyboardState keyboard, FrameEventArgs e)
        {
            // poll the mouse for events
            PollMouse(window, camera, mouse);

            // poll the keyboard for events
            PollKeyboard(window, camera, keyboard, e);

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
                var deltaX = mouse.X - _lastPos.X;
                var deltaY = mouse.Y - _lastPos.Y;

                // Apply the camera pitch and yaw (we clamp the pitch in the camera class)
                camera.Yaw += deltaX * camera.Sensitivity;
                camera.Pitch -= deltaY * camera.Sensitivity; // reversed since y-coordinates range from bottom to top
            }

            // updating last mouse position
            _lastPos = new Vector2(mouse.X, mouse.Y);

            // poll widget for interaction
            Widget.Poll(camera, mouse, MouseToNDC(window, mouse));
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
