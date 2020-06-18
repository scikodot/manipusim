using System;
using System.Security;
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
        public Model Model { get; private set; }
        public Collider Collider { get; private set; }

        public float Length => 2 * (Collider as CylinderCollider).HalfLength;  // TODO: perhaps optimize? or use another approach?

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

        public Link(LinkData data)
        {
            Model = data.Model;
            Collider = data.Collider;

            Collider.Body.UserObject = this;

            Model.RenderFlags = RenderFlags.Solid | RenderFlags.Wireframe | RenderFlags.Lighting;
        }

        public Link DeepCopy()
        {
            var link = (Link)MemberwiseClone();

            link.Model = Model.DeepCopy();
            link.Collider = Collider.DeepCopy();

            return link;
        }

        public bool CollisionTest()
        {
            // perform collision test with each obstacle
            bool collision = false;
            foreach (var obstacle in ObstacleHandler.Obstacles)
            {
                bool obstacleCollision = Collider.CollisionPairTest(obstacle.Collider);
                if (obstacleCollision)
                    collision = obstacleCollision;
            }

            // TODO: perform collision test with each link and gripper

            return collision;
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

        public void UpdateModel()  // TODO: unify
        {
            var state = Matrix.Scaling(Collider.Body.CollisionShape.LocalScaling) * State;
            Model.State = state.TopOpenTK();

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
