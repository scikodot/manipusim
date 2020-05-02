using System;
using System.Numerics;

using Graphics;

namespace Logic
{
    public enum JointType
    {
        Prismatic,  // Translation
        Revolute,  // Rotation
        Cylindrical,  // Translation & rotation
        Spherical,  // Allows three degrees of rotational freedom about the center of the joint. Also known as a ball-and-socket joint
        Planar  // Allows relative translation on a plane and relative rotation about an axis perpendicular to the plane
    }

    public struct JointData
    {
        public Model Model;
        public ICollidable Collider;

        public float Length;
        public float q;
        public Vector2 qRanges;
    }

    public class Joint
    {
        public Model Model { get; }
        public ICollidable Collider { get; }

        public float Length;

        public float q;
        public float[] qRanges;  // TODO: consider switching to Vector2 instead of array; array has a bit of overhead

        public ImpDualQuat State;

        public Vector3 Position { get; set; }
        public Vector3 Axis { get; set; }

        public Joint(JointData data)
        {
            Model = data.Model;
            Collider = data.Collider;

            Length = data.Length;
            q = data.q;
            qRanges = new float[2] { data.qRanges.X, data.qRanges.Y };
        }

        public Joint ShallowCopy()
        {
            return (Joint)MemberwiseClone();
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
