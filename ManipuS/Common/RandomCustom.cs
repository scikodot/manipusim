using System;
using Logic;

public enum GaussianAlgorithm
{
    BoxMullerTransform,
    RatioOfUniforms,
    Ziggurat
}

class RandomCustom : Random
{
    public RandomCustom() : base() { }

    public RandomCustom(int seed) : base(seed) { }

    public float NextGaussian(float mu, float sigma, GaussianAlgorithm algorithm = GaussianAlgorithm.RatioOfUniforms)
    {
        switch (algorithm)
        {
            case GaussianAlgorithm.BoxMullerTransform:
                return BoxMullerTransform.NextGaussian(this, mu, sigma);
            case GaussianAlgorithm.RatioOfUniforms:
                return RatioOfUniforms.NextGaussian(this, mu, sigma);
            case GaussianAlgorithm.Ziggurat:
                return Ziggurat.NextGaussian(this, mu, sigma);
            default:
                throw new ArgumentException("The specified algorithm does not exist!", "Algorithm");
        }
    }

    public static double Gaussian(double x)
    {
        return Math.Exp(-(x * x / 2));
    }

    public static double GaussianInv(double x)
    {
        return Math.Sqrt(-2 * Math.Log(x > 1 ? 1 : x));
    }
}
