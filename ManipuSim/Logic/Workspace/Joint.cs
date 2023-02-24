﻿using System;
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

    public class TranslationEventArgs : EventArgs
    {
        public Vector3 Translation { get; }

        public TranslationEventArgs(Vector3 translation)
        {
            Translation = translation;
        }
    }

    public class Joint : IDisposable, ISelectable, ITranslatable  // TODO: consider using abstract class Selectable instead of interface
    {
        public Model Model { get; private set; }
        public Collider Collider { get; private set; }

        public float Radius => (Collider as SphereCollider).Radius;

        private bool _showCollider;
        public ref bool ShowCollider => ref _showCollider;

        public Matrix State
        {
            get => Collider.Body.WorldTransform;
            set
            {
                // explicitly set position of the body
                Collider.Body.WorldTransform = value;

                // set its motion state to update position (for kinematic objects only)
                Collider.Body.MotionState.SetWorldTransform(ref value);
            }
        }

        private Vector3 _initialPosition;
        public ref Vector3 InitialPosition => ref _initialPosition;

        private Vector3 _initialAxis;
        public ref Vector3 InitialAxis => ref _initialAxis;

        public Vector3 Position { get; set; }
        public Vector3 Axis { get; set; }

        private float _initialCoordinate;
        public ref float InitialCoordinate => ref _initialCoordinate;

        private Vector2 _coordinateRange;
        public ref Vector2 CoordinateRange => ref _coordinateRange;

        public float Coordinate { get; set; }

        private bool _active;
        public ref bool Active => ref _active;

        public event EventHandler<TranslationEventArgs> TranslationChanged;

        public Joint(JointData data)
        {
            Model = data.Model;
            Collider = data.Collider;

            Collider.Body.UserObject = this;

            //Radius = data.Length;
            Coordinate = data.q;
            CoordinateRange = data.qRanges;

            Model.RenderFlags = RenderFlags.Solid | RenderFlags.Wireframe | RenderFlags.Lighting;
        }

        public void Translate(Vector3 translation)
        {
            State *= Matrix.Translation(translation.X, translation.Y, translation.Z);

            InitialPosition += translation;
            
            // invoke the event for the containing manipulator
            TranslationChanged?.Invoke(this, new TranslationEventArgs(translation));
        }

        public Joint DeepCopy()
        {
            var joint = (Joint)MemberwiseClone();

            joint.Model = Model.DeepCopy();
            joint.Collider = Collider.Copy();

            return joint;
        }

        public void Render(Shader shader, Action render = null)
        {
            Model.Render(shader, render);

            if (ShowCollider)
                Collider.Render(shader);
        }

        public void UpdateStateDesign()
        {
            Collider.Scale();
        }

        public void UpdateModel()  // TODO: unify
        {
            var state = Matrix.Scaling(Collider.Body.CollisionShape.LocalScaling) * State;
            Model.State = state.ToOpenTK();

            Collider.UpdateModel();
        }

        public void UpdateState(ref ImpDualQuat state)
        {
            State = state.ToMatrix().ToBullet();
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
