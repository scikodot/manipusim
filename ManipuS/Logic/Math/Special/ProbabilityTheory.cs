using System;

namespace Logic
{
    static class ProbabilityTheory
    {
        public static float BoxMullerTransform(float mu, float sigma)
        {
            double phi = Dispatcher.Rng.NextDouble();
            double r = 1 - Dispatcher.Rng.NextDouble();  // exclude zero
            float z = (float)(Math.Cos(2 * Math.PI * phi) * Math.Sqrt(-2 * Math.Log(r)));
            return mu + sigma * z;
        }
    }
}
