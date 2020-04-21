using System;

namespace Logic
{
    static class RatioOfUniforms
    {
        private const double eSqrt = 0.4288819424803534;  // Math.Sqrt(2 / Math.E);

        public static float NextGaussian(Random rng, float mu, float sigma)
        {
            double u1, u2;
            while (true)
            {
                u1 = 1 - rng.NextDouble();  // exclude zero
                u2 = eSqrt * (rng.NextDouble() - 0.5);

                if (-4 * u1 * u1 * Math.Log(u1) >= u2 * u2)
                    return mu + sigma * (float)(u2 / u1);
            }
        }
    }
}
