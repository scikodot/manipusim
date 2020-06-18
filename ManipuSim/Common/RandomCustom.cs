using System;
using System.Numerics;
using Logic;

class RandomCustom : Random
{
    public RandomCustom() : base() { }

    public RandomCustom(int seed) : base(seed) { }

    public int NextSign()
    {
        return Next(0, 2) == 0 ? -1 : 1;
    }

    public double NextDouble(double rangeLimit)
    {
        return NextDouble(-rangeLimit, rangeLimit);
    }

    public double NextDouble(double minValue, double maxValue)
    {
        return minValue + NextDouble() * (maxValue - minValue);
    }

    public float NextGaussian(float mu, float sigma)
    {
        return Ziggurat.NextGaussian(this, mu, sigma);
    }

    public Vector3 NextPointCube(float halfExtent)
    {
        return new Vector3((float)NextDouble(halfExtent), (float)NextDouble(halfExtent), (float)NextDouble(halfExtent));
    }

    public Vector3 NextPointSphere(float radius)
    {
        double x = NextDouble(radius);
        double yPositive = Math.Sqrt(radius * radius - x * x);
        double y = NextDouble(yPositive);
        double zPositive = Math.Sqrt(yPositive * yPositive - y * y);
        double z = NextDouble(zPositive);

        return new Vector3((float)x, (float)y, (float)z);
    }
}
