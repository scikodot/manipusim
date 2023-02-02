using OpenTK.Windowing.Desktop;

namespace Graphics
{
    // TODO: maintain proper transformation sequence: scale -> rotate -> translate; this may differ in different parts of program
    public static class Program
    {
        private static int _defaultWidth = 1200;
        private static int _defaultHeight = 900;

        private static void Main()
        {
            var gameWindowSettings = new GameWindowSettings()
            {
                RenderFrequency = 0,
                UpdateFrequency = 60
            };

            var nativeWindowSettings = new NativeWindowSettings()
            {
                Size = new OpenTK.Mathematics.Vector2i(_defaultWidth, _defaultHeight),
                Title = "ManipuSim",
                StartFocused = true  // TODO: works on Ubuntu, doesn't work on Windows 10; seems like a bug in source; report!
            };

            using (var window = new MainWindow(gameWindowSettings, nativeWindowSettings))
            {
                window.VSync = OpenTK.Windowing.Common.VSyncMode.Adaptive;
                window.Run();
            }
        }
    }
}