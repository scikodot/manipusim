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

    public Vector3 NextPoint3D(float range)
    {
        return new Vector3((float)NextDouble(range), (float)NextDouble(range), (float)NextDouble(range));
    }
}
