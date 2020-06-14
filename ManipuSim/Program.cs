using OpenToolkit.Windowing.Desktop;
using System;

namespace Graphics
{
    public static class Program
    {
        //[System.Runtime.InteropServices.DllImport("gdi32.dll")]
        //static extern int GetDeviceCaps(IntPtr hdc, int nIndex);
        //public enum DeviceCap
        //{
        //    VERTRES = 10,
        //    DESKTOPVERTRES = 117
        //}

        //private static float GetScalingFactor()
        //{
        //    System.Drawing.Graphics g = System.Drawing.Graphics.FromHwnd(IntPtr.Zero);
        //    IntPtr desktop = g.GetHdc();

        //    int LogicalScreenHeight = GetDeviceCaps(desktop, (int)DeviceCap.VERTRES);
        //    int PhysicalScreenHeight = GetDeviceCaps(desktop, (int)DeviceCap.DESKTOPVERTRES);

        //    float ScreenScalingFactor = (float)PhysicalScreenHeight / LogicalScreenHeight;

        //    return ScreenScalingFactor;
        //}

        private static void Main()
        {
            int desWidth = /*1600*/1200, desHeight = /*750*/900;

            //// Retrieve screen scaling factor
            //float scale = GetScalingFactor();

            //// Scale window size so that it is displayed properly
            //desWidth = (int)(desWidth / scale);
            //desHeight = (int)(desHeight / scale);

            //GraphicsMode mode = new GraphicsMode(new ColorFormat(24), 16, 8, 4, new ColorFormat(32), 2, false);
            //using (var window = new MainWindow(desWidth, desHeight, mode, "ManipuSim"))
            //{
            //    window.Run(60.0);
            //}

            //var gameWindowSettings = new GameWindowSettings()
            //{ 
            //    RenderFrequency = 60
            //};
            var gameWindowSettings = GameWindowSettings.Default;

            var nativeWindowSettings = new NativeWindowSettings()
            {
                Size = new OpenToolkit.Mathematics.Vector2i(desWidth, desHeight),
                Title = "ManipuSim",
                StartFocused = true  // TODO: this doesn't work for some reason (seems like a bug in source)
            };

            using (var window = new MainWindow(gameWindowSettings, nativeWindowSettings))
            {
                window.Run();
            }
        }
    }
}