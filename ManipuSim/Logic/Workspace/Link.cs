using System;
using System.Threading.Tasks;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Physics;

namespace Logic
{
    public class Link : IDisposable, ISelectable
    {
        private static Model _defaultModel;

        public Model Model { get; private set; }
        public Collider Collider { get; private set; }

        public Matrix State => Collider.State;

        public float Length => 2 * (Collider as CylinderCollider).HalfLength;  // TODO: perhaps optimize? or use another approach?
        public bool ShowCollider { get; set; }

        public static void LoadDefaultModel(string path)
        {
            Dispatcher.ActiveTasks.Add(Task.Run(() =>
            {
                _defaultModel = new Model(path);
            }));
        }

        public Link(Model model = null, Collider collider = null, float length = 0.5f)
        {
            Model = model ?? _defaultModel.Copy();
            Collider = collider ?? Collider.Create(new CylinderShape(0.15f, length, 0.15f));
            Collider.Body.UserObject = this;

            Model.RenderFlags = RenderFlags.Default | RenderFlags.Wireframe | RenderFlags.Lighting;
        }

        public Link Copy() => new(Model.Copy(), Collider.Copy());

        public bool CollisionTest()
        {
            /*// perform collision test with each obstacle
            bool collision = false;
            foreach (var obstacle in ObstacleHandler.Obstacles)
            {
                bool obstacleCollision = Collider.CollisionPairTest(obstacle.Collider);
                if (obstacleCollision)
                    collision = obstacleCollision;
            }

            // TODO: perform collision test with each link and gripper

            return collision;*/

            return default;
        }

        public void Render(ShaderProgram shader)
        {
            Model.Render(shader);

            if (ShowCollider)
                Collider.Render(shader);
        }

        public void Update(InteractionMode mode)
        {
            //Collider.Scale();
            var state = State.ToOpenTK();
            Model.Update(state);
            Collider.Model.Update(state);
        }

        public void UpdateState(ref ImpDualQuat state)
        {
            State = state.ToMatrix().ToBullet();
        }

        public void Dispose()
        {
            Model.Dispose();
            Collider.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
