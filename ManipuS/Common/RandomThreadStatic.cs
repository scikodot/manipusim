using System;

public static class RandomThreadStatic
{
    private static Random _global = new Random();
    [ThreadStatic]
    private static RandomCustom _instance;

    private static RandomCustom Instance
    {
        get
        {
            if (_instance == null)
                lock (_global)
                    _instance = new RandomCustom(_global.Next());

            return _instance;
        }
    }

    public static int Next()
    {
        return Instance.Next();
    }

    public static int Next(int maxValue)
    {
        return Instance.Next(maxValue);
    }

    public static int Next(int minValue, int maxValue)
    {
        return Instance.Next(minValue, maxValue);
    }

    public static double NextDouble()
    {
        return Instance.NextDouble();
    }

    public static double NextDouble(double rangeLimit)
    {
        return NextDouble(-rangeLimit, rangeLimit);
    }

    public static double NextDouble(double minValue, double maxValue)
    {
        return minValue + Instance.NextDouble() * (maxValue - minValue);
    }

    public static float NextGaussian(float mu, float sigma)
    {
        return Instance.NextGaussian(mu, sigma);
    }
}
