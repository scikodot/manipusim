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
}
