using OpenTK.Graphics;
using System;

namespace Graphics
{
    public static class Program
    {
        [System.Runtime.InteropServices.DllImport("gdi32.dll")]
        static extern int GetDeviceCaps(IntPtr hdc, int nIndex);
        public enum DeviceCap
        {
            VERTRES = 10,
            DESKTOPVERTRES = 117
        }

        private static float GetScalingFactor()
        {
            System.Drawing.Graphics g = System.Drawing.Graphics.FromHwnd(IntPtr.Zero);
            IntPtr desktop = g.GetHdc();

            int LogicalScreenHeight = GetDeviceCaps(desktop, (int)DeviceCap.VERTRES);
            int PhysicalScreenHeight = GetDeviceCaps(desktop, (int)DeviceCap.DESKTOPVERTRES);

            float ScreenScalingFactor = (float)PhysicalScreenHeight / LogicalScreenHeight;

            return ScreenScalingFactor;
        }

        private static void Main()
        {
            int desWidth = 1200, desHeight = 800;

            // Retrieve screen scaling factor
            float scale = GetScalingFactor();

            // Scale window size so that it is displayed properly
            desWidth = (int)(desWidth / scale);
            desHeight = (int)(desHeight / scale);

            GraphicsMode mode = new GraphicsMode(new ColorFormat(24), 16, 8, 4, new ColorFormat(32), 2, false);
            using (var window = new Window(desWidth, desHeight, "ManipuS", mode))
            {
                window.Run(60.0);
            }
        }
    }
}