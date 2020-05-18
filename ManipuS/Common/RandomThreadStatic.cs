using System;
using System.Numerics;

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

    public static int NextSign()
    {
        return Instance.NextSign();
    }

    public static double NextDouble()
    {
        return Instance.NextDouble();
    }

    public static double NextDouble(double rangeLimit)
    {
        return Instance.NextDouble(rangeLimit);
    }

    public static double NextDouble(double minValue, double maxValue)
    {
        return Instance.NextDouble(minValue, maxValue);
    }

    public static float NextGaussian(float mu, float sigma)
    {
        return Instance.NextGaussian(mu, sigma);
    }

    public static Vector3 NextPoint3D(float range)
    {
        return Instance.NextPoint3D(range);
    }
}
