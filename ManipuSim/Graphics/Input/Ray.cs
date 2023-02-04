using OpenTK.Mathematics;

namespace Graphics
{
    public class Ray
    {
        private Vector4 _startWorld;
        public ref Vector4 StartWorld => ref _startWorld;

        private Vector4 _endWorld;
        public ref Vector4 EndWorld => ref _endWorld;

        private Vector4 _direction;
        public ref Vector4 Direction => ref _direction;

        private Ray(Vector4 rayStartWorld, Vector4 rayEndWorld, Vector4 rayDir)
        {
            _startWorld = rayStartWorld;
            EndWorld = rayEndWorld;
            Direction = rayDir;
        }

        public static Ray Cast(Camera camera) => Cast(ref camera.ViewMatrix, ref camera.ProjectionMatrix);

        private static Ray Cast(ref Matrix4 view, ref Matrix4 proj)  // TODO: casting is performed inaccurately; fix, optimize
        {
            var cursorPos = InputHandler.CursorPositionNDC;

            var pointStart = new Vector4(cursorPos.X, cursorPos.Y, -1, 1);
            var pointEnd = new Vector4(cursorPos.X, cursorPos.Y, 0, 1);

            var projViewInv = Matrix4.Invert(view * proj);

            var rayStartWorld = pointStart * projViewInv;
            rayStartWorld /= rayStartWorld.W;

            var rayEndWorld = pointEnd * projViewInv;
            rayEndWorld /= rayEndWorld.W;

            var rayDir = Vector4.Normalize(rayEndWorld - rayStartWorld);
            rayEndWorld = rayStartWorld + rayDir * 1000;

            return new Ray(rayStartWorld, rayEndWorld, rayDir);
        }
    }
}
