using OpenToolkit.Windowing.Desktop;

namespace Graphics
{
    // TODO: maintain proper transformation sequence: scale -> rotate -> translate; this may differ in different parts of program
    public static class Program
    {
        private static void Main()
        {
            int desWidth = 1200, desHeight = 900;

            var gameWindowSettings = new GameWindowSettings()
            {
                RenderFrequency = 0,
                UpdateFrequency = 60
            };

            var nativeWindowSettings = new NativeWindowSettings()
            {
                Size = new OpenToolkit.Mathematics.Vector2i(desWidth, desHeight),
                Title = "ManipuSim",
                StartFocused = true  // TODO: works on Ubuntu, doesn't work on Windows 10; seems like a bug in source; report!
            };

            using (var window = new MainWindow(gameWindowSettings, nativeWindowSettings))
            {
                window.VSync = OpenToolkit.Windowing.Common.VSyncMode.Adaptive;
                window.Run();
            }
        }
    }
}