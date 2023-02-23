using OpenTK.Mathematics;

namespace Graphics
{
    public readonly struct Ray
    {
        // ray properties
        public Vector4 StartWorld { get; }
        public Vector4 EndWorld { get; }
        public Vector4 Direction { get; }

        // minimal necessary camera state info
        public Vector3 CameraPosition { get; }
        public Vector3 CameraFront { get; }

        private Ray(Camera camera, Vector4 startWorld, Vector4 endWorld, Vector4 direction)
        {
            StartWorld = startWorld;
            EndWorld = endWorld;
            Direction = direction;
            CameraPosition = camera.Position;
            CameraFront = camera.Front;
        }

        // cast a ray from the given camera view
        public static Ray Cast(Camera camera, Vector2 cursorPosition)  // TODO: casting is performed inaccurately; fix, optimize
        {
            var pointStart = new Vector4(cursorPosition.X, cursorPosition.Y, -1, 1);
            var pointEnd = new Vector4(cursorPosition.X, cursorPosition.Y, 0, 1);

            var projViewInv = Matrix4.Invert(camera.ViewMatrix * camera.ProjectionMatrix);

            var startWorld = pointStart * projViewInv;
            startWorld /= startWorld.W;

            var endWorld = pointEnd * projViewInv;
            endWorld /= endWorld.W;

            var direction = Vector4.Normalize(endWorld - startWorld);
            endWorld = startWorld + direction * 1000;

            return new Ray(camera, startWorld, endWorld, direction);
        }
    }
}
