using System;

namespace Logic
{
    static class Ziggurat
    {
        private static uint[] k;
        private static double[] w;
        private static double[] f;

        static Ziggurat()
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
                funcVal = RandomCustom.Gaussian(xCurr);
                xPrev = RandomCustom.GaussianInv(v / xCurr + funcVal);

                k[i] = (uint)(uint.MaxValue * (xPrev / xCurr));
                w[i] = xCurr / uint.MaxValue;
                f[i] = funcVal;

                xCurr = xPrev;
            }

            funcVal = RandomCustom.Gaussian(r);
            k[0] = (uint)(uint.MaxValue * (r * funcVal / v));
            w[0] = v / (funcVal * uint.MaxValue);
        }

        public static float NextGaussian(Random rng, float mu, float sigma)
        {
            int j;
            int i;
            double x;
            while (true)
            {
                int highHalf = rng.Next(1 << 16) << 16;
                int lowHalf = rng.Next(1 << 16);
                j = highHalf | lowHalf;
                i = j & 255;

                x = j * w[i];
                if (Math.Abs(j) < k[i])
                    return mu + sigma * (float)x;

                if (i == 0)
                {
                    // return an x from the tail
                    continue;  // TODO: implement
                }

                if (Dispatcher.Rng.NextDouble() * (f[i - 1] - f[i]) < RandomCustom.Gaussian(x) - f[i])
                    return mu + sigma * (float)x;
            }
        }
    }
}
