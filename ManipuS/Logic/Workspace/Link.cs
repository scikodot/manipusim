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

    public class Link
    {
        public Model Model { get; }
        public Collider Collider { get; }

        public float Length;  // TODO: must be names Size or something like that; Length is not suitable

        public Matrix State
        {
            get
            {
                Collider.Body.MotionState.GetWorldTransform(out Matrix state);
                return state;
            }
            set
            {
                Collider.Body.MotionState.SetWorldTransform(ref value);
            }
        }

        public Link(LinkData data)
        {
            Model = data.Model;
            Collider = data.Collider;

            Length = data.Length;
        }

        public Link ShallowCopy()
        {
            return (Link)MemberwiseClone();
        }

        public void Render(Shader shader, MeshMode mode, Action render = null, bool showCollider = false)
        {
            Model.Render(shader, mode, render);

            if (showCollider)
                Collider.Render(shader);
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
