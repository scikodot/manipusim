using System;
using Logic;

class RandomCustom : Random
{
    public RandomCustom() : base() { }

    public RandomCustom(int seed) : base(seed) { }

    public float NextGaussian(float mu, float sigma)
    {
        return Ziggurat.NextGaussian(this, mu, sigma);
    }
}
