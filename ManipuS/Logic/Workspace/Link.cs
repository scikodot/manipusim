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

    public class Link : ISelectable
    {
        public Model Model { get; }
        public Collider Collider { get; }

        public float Length => 2 * (Collider as CylinderCollider).HalfLength;  // TODO: perhaps optimize? or use another approach?

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

        public void Render(Shader shader, Action render = null, bool showCollider = false)
        {
            UpdateState();

            Model.Render(shader, render);

            if (showCollider)
                Collider.Render(shader);
        }

        public void UpdateStateDesign()
        {
            Collider.Scale();
        }

        public void UpdateState()
        {
            var state = Matrix.Scaling(Collider.Body.CollisionShape.LocalScaling) * State;

            OpenTK.Matrix4 stateMatrix = new OpenTK.Matrix4(
                state.M11, state.M21, state.M31, state.M41,
                state.M12, state.M22, state.M32, state.M42,
                state.M13, state.M23, state.M33, state.M43,
                state.M14, state.M24, state.M34, state.M44);

            Model.State = stateMatrix;
            Collider.UpdateState(ref stateMatrix);
        }

        public void UpdateState(ref ImpDualQuat state)
        {
            var stateMatrix = state.ToBulletMatrix();
            State = stateMatrix;

            OpenTK.Matrix4 stateMatrixOpenTK = new OpenTK.Matrix4(
                stateMatrix.M11, stateMatrix.M21, stateMatrix.M31, stateMatrix.M41,
                stateMatrix.M12, stateMatrix.M22, stateMatrix.M32, stateMatrix.M42,
                stateMatrix.M13, stateMatrix.M23, stateMatrix.M33, stateMatrix.M43,
                stateMatrix.M14, stateMatrix.M24, stateMatrix.M34, stateMatrix.M44);

            Model.State = stateMatrixOpenTK;
            Collider.UpdateState(ref stateMatrixOpenTK);
        }
    }
}
