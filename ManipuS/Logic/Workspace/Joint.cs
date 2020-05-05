using System;
using System.Numerics;
using BulletSharp.Math;
using Graphics;
using Physics;

using Vector3 = System.Numerics.Vector3;

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
        public Collider Collider;

        public float Length;
        public float q;
        public Vector2 qRanges;
    }

    public class Joint
    {
        public Model Model { get; }
        public Collider Collider { get; }

        public float Length;

        public float q;
        public float[] qRanges;  // TODO: consider switching to Vector2 instead of array; array has a bit of overhead

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
