using System;

namespace Logic
{
    static class RatioOfUniforms
    {
        private static double eSqrt = Math.Sqrt(2 / Math.E);
        private static double eSqrt2 = 2 * eSqrt;

        public static float NextGaussian(Random rng, float mu, float sigma)
        {
            double u1, u2;
            while (true)
            {
                u1 = 1 - rng.NextDouble();  // exclude zero
                u2 = eSqrt2 * rng.NextDouble() - eSqrt;

                if (-4 * u1 * u1 * Math.Log(u1) >= u2 * u2)
                    return mu + sigma * (float)(u2 / u1);
            }
        }
    }
}
