namespace RoboDraw
{
    public static class Program
    {
        private static void Main()
        {
            using (var window = new Window(800, 600, "SafePathSearch"))
            {
                window.Run(60.0);
            }
        }
    }
}