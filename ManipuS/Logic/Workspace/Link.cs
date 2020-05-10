using System;
using BulletSharp.Math;
using Graphics;
using Physics;

namespace Logic
{
    public struct LinkData
    {
        public Model Model;
        public Collider Collider;

        public float Length;
    }

    public class Link : IDisposable, ISelectable
    {
        public Model Model { get; }
        public Collider Collider { get; }

        public float Length => 2 * (Collider as CylinderCollider).HalfLength;  // TODO: perhaps optimize? or use another approach?

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        public Matrix State
        {
            get => Collider.Body.MotionState.WorldTransform;
            set => Collider.Body.MotionState.WorldTransform = value;
        }

        public Link(LinkData data)
        {
            Model = data.Model;
            Collider = data.Collider;

            Collider.Body.UserObject = this;

            //Length = data.Length;

            Model.RenderFlags = RenderFlags.Solid | RenderFlags.Wireframe | RenderFlags.Lighting;
        }

        public Link ShallowCopy()
        {
            return (Link)MemberwiseClone();
        }

        public void Render(Shader shader, Action render = null)
        {
            Model.Render(shader, render);

            if (_showCollider)
                Collider.Render(shader);
        }

        public void UpdateStateDesign()
        {
            Collider.Scale();
        }

        public void UpdateModel()
        {
            var state = Matrix.Scaling(Collider.Body.CollisionShape.LocalScaling) * State;

            OpenTK.Matrix4 stateMatrix = new OpenTK.Matrix4(
                state.M11, state.M21, state.M31, state.M41,
                state.M12, state.M22, state.M32, state.M42,
                state.M13, state.M23, state.M33, state.M43,
                state.M14, state.M24, state.M34, state.M44);

            Model.State = stateMatrix;
            Collider.UpdateModel(ref stateMatrix);
        }

        public void UpdateState(ref ImpDualQuat state)
        {
            State = state.ToBulletMatrix();
        }

        public void Dispose()
        {
            // clear managed resources
            Model.Dispose();
            Collider.Dispose();

            // suppress finalization
            GC.SuppressFinalize(this);
        }
    }
}
