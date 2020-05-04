using System;

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

        public ImpDualQuat State;

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
            State = state;

            OpenTK.Matrix4 stateMatrix = state.ToMatrix();
            Model.State = stateMatrix;
            Collider.UpdateState(ref stateMatrix);
        }
    }
}
