namespace Graphics
{
    public static class Utils
    {
        public static float[] GLConvert(System.Numerics.Vector3[] data, OpenTK.Graphics.Color4 color)  // TODO: decide where to place
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

        public static float[] GLConvert(System.Numerics.Vector3[] data)  // TODO: decide where to place
        {
            // converting program data to OpenGL buffer format
            float[] res = new float[data.Length * 6];

            for (int i = 0; i < data.Length; i++)
            {
                res[6 * i] = data[i].X;
                res[6 * i + 1] = data[i].Y;
                res[6 * i + 2] = data[i].Z;
                res[6 * i + 3] = 0;
                res[6 * i + 4] = 0;
                res[6 * i + 5] = 0;
            }

            return res;
        }
    }
}
