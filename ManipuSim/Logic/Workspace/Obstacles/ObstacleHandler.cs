using System;
using System.Collections.Generic;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Physics;
using System.Linq;

namespace Logic
{
    public enum ObstacleShape  // TODO: for scalability enum should be replaced with dictionary (?) to enable adding custom derived classes
    {
        Box,
        Sphere,
        Cylinder,
        Cone
    }

    public class ObstacleHandler
    {
        private readonly MainWindow _parent;
        private readonly Ground _ground;
        private readonly MeshMaterial _defaultMaterial = new()
        {
            Ambient = new OpenTK.Mathematics.Color4(0.1f, 0.1f, 0.0f, 1.0f),
            Diffuse = new OpenTK.Mathematics.Color4(0.8f, 0.8f, 0.0f, 1.0f),
            Specular = new OpenTK.Mathematics.Color4(0.5f, 0.5f, 0.0f, 1.0f),
            Shininess = 8
        };

        public string[] Shapes { get; } = Enum.GetNames(typeof(ObstacleShape));
        public List<Obstacle> Obstacles { get; } = new();

        public ObstacleHandler(MainWindow parent)
        {
            _parent = parent;

            _ground = new Ground(_parent.PhysicsHandler.CreateCollider(
                RigidBodyType.Static, 
                new BoxShape(10, 0.2f, 10), 
                Matrix.Translation(-0.2f * Vector3.UnitY)));
        }

        public void Add(params Obstacle[] obstacles)
        {
            if (obstacles == null)
                throw new ArgumentNullException("obstacles");

            foreach (var obstacle in obstacles)
                Add(obstacle);
        }

        public void Add(Obstacle obstacle)
        {
            if (obstacle == null)
                return;

            Obstacles.Add(obstacle);
        }

        public void AddDefault(ObstacleShape shape)
        {
            var obstacle = shape switch
            {
                ObstacleShape.Box => new Obstacle(
                    new Model(new Mesh[]
                    {
                        Primitives.Cube(0.5f, _defaultMaterial)
                    }),
                    _parent.PhysicsHandler.CreateCollider(RigidBodyType.Kinematic, new BoxShape(0.5f))),
                ObstacleShape.Sphere => new Obstacle(
                    new Model(new Mesh[]
                    {
                        Primitives.Sphere(0.5f, 50, 50, _defaultMaterial)
                    }),
                    _parent.PhysicsHandler.CreateCollider(RigidBodyType.Kinematic, new SphereShape(0.5f))),
                ObstacleShape.Cylinder => new Obstacle(
                    new Model(new Mesh[]
                    {
                        Primitives.Cylinder(0.25f, 1f, 50, _defaultMaterial)
                    }),
                    _parent.PhysicsHandler.CreateCollider(RigidBodyType.Kinematic, new CylinderShape(0.25f, 1f, 0.25f))),
                ObstacleShape.Cone => new Obstacle(
                    new Model(new Mesh[]
                    {
                        Primitives.Cone(0.5f, 2, 50, _defaultMaterial)
                    }),
                    _parent.PhysicsHandler.CreateCollider(RigidBodyType.Kinematic, new ConeShape(0.5f, 2))),
                _ => throw new ArgumentException("Unknown obstacle shape.")
            };

            Add(obstacle);
        }

        public void Remove(Obstacle obstacle)
        {
            if (Obstacles.Remove(obstacle))
                obstacle.Dispose();
        }

        // TODO: perhaps return all containing obstacles?
        public bool ContainmentTest(Vector3 point, out Obstacle container)
        {
            foreach (var obstacle in Obstacles)
            {
                if (obstacle.Contains(point))
                {
                    container = obstacle;
                    return true;
                }
            }

            container = null;
            return false;
        }

        public void Update()
        {
            foreach (var obstacle in Obstacles)
                obstacle.Update(_parent.InputHandler.InteractionMode);
        }

        public void OnInteractionModeSwitched(InteractionModeSwitchEventArgs e)
        {
            foreach (var obstacle in Obstacles)
            {
                obstacle.OnInteractionModeSwitched(e);
            }
        }

        public void RenderGrid(Shader shader)
        {
            _ground.RenderGrid(shader);
        }

        public void RenderGround(Shader shader)
        {
            _ground.RenderGround(shader);
        }

        public void RenderUnselected(Shader shader)
        {
            foreach (var obstacle in Obstacles)
            {
                //if (obstacle.Type == RigidBodyType.Kinematic)
                //    obstacle.Path.Model.Render(shader);

                if (!obstacle.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    obstacle.Render(shader);
            }
        }

        public void RenderSelected(Shader shader)
        {
            foreach (var obstacle in Obstacles)
            {
                if (obstacle.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    obstacle.Render(shader);
            }
        }

        public void Dispose()
        {
            // dispose of the ground
            _ground.Dispose();

            // dispose of all the obstacles
            foreach (var obstacle in Obstacles)
                obstacle.Dispose();
        }
    }
}
