using System;

namespace Logic
{
    static class Ziggurat
    {
        // normal distribution constants
        private const double r = 3.6541528853610088;  // position of the right-most rectangle right side
        private const double rInv = 1 / r;  // inverse of r
        private const double v = 0.00492867323399;  // area of each rectangle

        // algorithm parameters
        private readonly static uint[] k;
        private readonly static double[] w;
        private readonly static double[] f;

        static Ziggurat()
        {
            // pre-compute all algorithm parameters
            k = new uint[256];
            w = new double[256];
            f = new double[256];

            double xCurr = r, xPrev, gaussValue;
            for (int i = 255; i > 0; i--)
            {
                gaussValue = Gaussian(xCurr);
                xPrev = GaussianInv(v / xCurr + gaussValue);
                k[i] = (uint)(uint.MaxValue * (xPrev / xCurr));
                w[i] = xCurr / uint.MaxValue;
                f[i] = gaussValue;

                xCurr = xPrev;
            }

            gaussValue = Gaussian(r);
            k[0] = (uint)(uint.MaxValue * (r * gaussValue / v));
            w[0] = v / (gaussValue * uint.MaxValue);
            f[0] = 1;
        }

        public static float NextGaussian(Random rng, float mu, float sigma)
        {
            int i, j;
            double x, y;
            while (true)
            {
                // get random point
                j = rng.Next(int.MinValue, int.MaxValue);

                // get random layer
                i = j & 255;

                // find position of the point on that layer
                x = j * w[i];

                // return the current point if it definitely falls under the distribution curve
                if (Math.Abs(j) < k[i])
                    return mu + sigma * (float)x;

                // else, if the point is not definitely under the curve and the current layer is the tail,
                // return x from the tail with Marsaglia's method
                if (i == 0)
                {
                    do
                    {
                        x = -Math.Log(rng.NextDouble()) * rInv;
                        y = -Math.Log(rng.NextDouble());
                    } while (y + y < x * x);

                    return (float)(j > 0 ? r + x : -r - x);
                }

                // else, the point is in a boundary region,
                // hence do an additional check for whether the point is under the curve or not
                if (rng.NextDouble() * (f[i - 1] - f[i]) < Gaussian(x) - f[i])
                    return mu + sigma * (float)x;
            }
        }

        private static double Gaussian(double x)
        {
            return Math.Exp(-(x * x * 0.5));
        }

        private static double GaussianInv(double x)
        {
            // due to floating point error, x can become greater than 1;
            // this leads to negative number under the root;
            // additional check for 1 is done to prevent it
            return Math.Sqrt(-2 * Math.Log(x > 1 ? 1 : x));
        }
    }
}
