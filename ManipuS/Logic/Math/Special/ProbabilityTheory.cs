using System;

namespace Logic
{
    static class ProbabilityTheory
    {
        private static uint[] k;
        private static double[] w;
        private static double[] f;

        static ProbabilityTheory()
        {
            k = new uint[256];
            w = new double[256];
            f = new double[256];

            double r = 3.6541528853610088;
            double v = 0.00492867323399;

            double xCurr = r;
            double xPrev;
            double funcVal;
            for (int i = 255; i > 0; i--)
            {
                funcVal = func(xCurr);
                xPrev = funcInv(v / xCurr + funcVal);

                k[i] = (uint)(uint.MaxValue * (xPrev / xCurr));
                w[i] = xCurr / uint.MaxValue;
                f[i] = funcVal;

                xCurr = xPrev;
            }

            funcVal = func(r);
            k[0] = (uint)(uint.MaxValue * (r * funcVal / v));
            w[0] = v / (func(r) * uint.MaxValue);
        }

        static double func(double x) => Math.Exp(-x * x / 2);
        static double funcInv(double x) => Math.Sqrt(-2 * Math.Log(x > 1 ? 1 : x));

        public static float RandomGaussian(float mu, float sigma)
        {
            //// Box-Muller transform
            //double phi = Dispatcher.Rng.NextDouble();
            //double r = 1 - Dispatcher.Rng.NextDouble();  // exclude zero
            //double z = Math.Cos(2 * Math.PI * phi) * Math.Sqrt(-2 * Math.Log(r));
            //return mu + sigma * (float)z;

            // Ziggurat algorithm  // TODO: something's wrong with the distribution (too low mean-biasing); check
            uint j;
            uint i;
            double x;
            while (true)
            {
                uint highHalf = (uint)Dispatcher.Rng.Next(1 << 16) << 16;
                uint lowHalf = (uint)Dispatcher.Rng.Next(1 << 16);
                j = highHalf | lowHalf;
                i = j & 255;

                x = j * w[i];
                if (j < k[i])
                    return mu + sigma * (float)x;

                if (i == 0)
                {
                    // return an x from the tail
                    continue;  // TODO: implement
                }

                if (Dispatcher.Rng.NextDouble() * (f[i - 1] - f[i]) < func(x) - f[i])
                    return mu + sigma * (float)x;
            }
        }
    }
}
