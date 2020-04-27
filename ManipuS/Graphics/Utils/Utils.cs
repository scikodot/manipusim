using System.Threading.Tasks;
using System.Drawing.Imaging;

using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL4;
using System.Drawing;
using System.Linq;

namespace Graphics
{
    public static class Utils
    {
        public static void ScreenCapture(Window win, string savepath)
        {
            // taking a picture of a viewport
            byte[,,] img = new byte[win.Height, win.Width, 3];
            GL.ReadPixels(0, 0, win.Width, win.Height, OpenTK.Graphics.OpenGL4.PixelFormat.Rgb, PixelType.UnsignedByte, img);

            Task.Run(() =>
            {
                // create a bitmap representing captured screenshot
                var bitmap = new Bitmap(win.Width, win.Height);

                // write all captured data to that bitmap
                for (int i = 0; i < win.Height; i++)
                {
                    for (int j = 0; j < win.Width; j++)
                    {
                        bitmap.SetPixel(j, win.Height - 1 - i, Color.FromArgb(img[i, j, 0], img[i, j, 1], img[i, j, 2]));
                    }
                }

                // get the image format
                string savepathLow = savepath.ToLower();
                string formatString = "";
                for (int i = savepathLow.Length - 1; i >= 0; i--)
                {
                    formatString += savepathLow[i];

                    if (savepathLow[i] == '.' || savepathLow[i] == '/' || savepathLow[i] == '\\')
                        break;
                }
                formatString = new string(formatString.Reverse().ToArray());

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
                    bitmap.Save(savepath, format);
                else
                    bitmap.Save(savepath + ".bmp", ImageFormat.Bmp);
            });
        }

        public static float[] GL_Convert(System.Numerics.Vector3[] data, Color4 color)
        {
            // converting program data to OpenGL buffer format
            float[] res = new float[data.Length * 7];

            for (int i = 0; i < data.Length; i++)
            {
                res[7 * i] = data[i].X;
                res[7 * i + 1] = data[i].Y;
                res[7 * i + 2] = data[i].Z;
                res[7 * i + 3] = color.R;
                res[7 * i + 4] = color.G;
                res[7 * i + 5] = color.B;
                res[7 * i + 6] = color.A;
            }

            return res;
        }
    }
}
