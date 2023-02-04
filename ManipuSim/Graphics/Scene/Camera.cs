using System;
using System.Net.Http.Headers;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.GraphicsLibraryFramework;

namespace Graphics
{
    public class Camera
    {
        // camera params
        public float Speed { get; private set; } = 3f;
        public float Sensitivity { get; private set; } = 0.2f;

        // world position of the camera
        public Vector3 Position { get; set; }

        // world orientation of the camera
        private Vector3 _front = -Vector3.UnitZ;
        public Vector3 Front => _front;

        private Vector3 _up = Vector3.UnitY;
        public Vector3 Up => _up;

        private Vector3 _right = Vector3.UnitX;
        public Vector3 Right => _right;

        // rotation around the X axis (radians)
        private float _pitch;
        public float Pitch
        {
            get => MathHelper.RadiansToDegrees(_pitch);
            // the pitch is clamped to avoid the "gimbal lock" behaviour
            private set => _pitch = MathHelper.DegreesToRadians(MathHelper.Clamp(value, -89f, 89f));
        }

        // rotation around the Y axis (radians)
        private float _yaw = -MathHelper.PiOver2;
        public float Yaw
        {
            get => MathHelper.RadiansToDegrees(_yaw);
            private set => _yaw = MathHelper.DegreesToRadians(value);
        }

        // field of view, i.e. a vertical angle (radians)
        private float _fov = MathHelper.PiOver3;
        public float FOV
        {
            get => MathHelper.RadiansToDegrees(_fov);
            private set => _fov = MathHelper.DegreesToRadians(MathHelper.Clamp(value, 1f, 60f));
        }

        private Matrix4 _viewMatrix;
        public ref Matrix4 ViewMatrix => ref _viewMatrix;
        public void UpdateViewMatrix() => 
            _viewMatrix = Matrix4.LookAt(Position, Position + _front, _up);

        private Matrix4 _projectionMatrix;
        public ref Matrix4 ProjectionMatrix => ref _projectionMatrix;
        private void UpdateProjectionMatrix(float aspect) => 
            _projectionMatrix = Matrix4.CreatePerspectiveFieldOfView(_fov, aspect, 0.01f, 100f);

        public Camera(Vector3 position, Vector2 orientation, float aspectRatio)
        {
            Position = position;
            Pitch = orientation.X;
            Yaw = orientation.Y;

            UpdateViewMatrix();
            UpdateProjectionMatrix(aspectRatio);
        }

        public void OnCameraMove(CameraMoveEventArgs e)
        {
            // position offset = direction * distance
            Position += (e.Direction.X * Right + e.Direction.Y * Up) * (Speed * e.Time);

            UpdateViewMatrix();
        }
        public void OnCameraRotate(CameraRotateEventArgs e)
        {
            // update angles; pitch is decreased because Y axis is pointing down in NDC
            Yaw += e.Delta.X * Sensitivity;
            Pitch -= e.Delta.Y * Sensitivity;

            // rotate the Z axis vector and normalize
            _front.X = (float)Math.Cos(_pitch) * (float)Math.Cos(_yaw);
            _front.Y = (float)Math.Sin(_pitch);
            _front.Z = (float)Math.Cos(_pitch) * (float)Math.Sin(_yaw);
            _front = Vector3.Normalize(_front);

            // fill up the basis
            _right = Vector3.Normalize(Vector3.Cross(_front, Vector3.UnitY));
            _up = Vector3.Normalize(Vector3.Cross(_right, _front));

            UpdateViewMatrix();
        }

        public void OnCameraZoom(CameraZoomEventArgs e)
        {
            FOV -= e.Delta;

            UpdateProjectionMatrix(e.AspectRatio);
        }
    }
}