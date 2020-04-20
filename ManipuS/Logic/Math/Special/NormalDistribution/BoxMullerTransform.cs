using System;

namespace Logic
{
    static class BoxMullerTransform
    {
        private static double pi2 = 2 * Math.PI;

        public static float NextGaussian(Random rng, float mu, float sigma)
        {
            double phi = rng.NextDouble();
            double r = 1 - rng.NextDouble();  // exclude zero
            double z = Math.Cos(pi2 * phi) * Math.Sqrt(-2 * Math.Log(r));
            return mu + sigma * (float)z;
        }
    }
}
