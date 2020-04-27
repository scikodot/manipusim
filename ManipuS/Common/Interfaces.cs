using OpenTK;

namespace Graphics
{
    public interface IRenderable
    {
        ref Matrix4 State { get; }
    }
}
