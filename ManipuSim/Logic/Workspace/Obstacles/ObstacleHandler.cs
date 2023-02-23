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
        private class Ground
        {
            private readonly Model _grid = Primitives.Grid(21, 1, MeshMaterial.White);

            private readonly Model _ground = Primitives.Plane(10, 10, new MeshMaterial
            {
                Diffuse = new OpenTK.Mathematics.Vector4(1.0f, 1.0f, 1.0f, 0.5f)
            });

            private readonly Collider _collider = PhysicsHandler.CreateStaticCollider(
                new BoxShape(10, 0.2f, 10),
                Matrix.Translation(-0.2f * BulletSharp.Math.Vector3.UnitY));

            public void RenderGround(Shader shader)
            {
                _ground.Render(shader);
            }

            public void RenderGrid(Shader shader)
            {
                _grid.Render(shader, () =>
                {
                    GL.DrawElements(BeginMode.Lines, _grid.Meshes[0].Indices.Length, DrawElementsType.UnsignedInt, 0);
                });
            }

            public void Dispose()
            {
                // clear managed resources
                _grid.Dispose();
                _ground.Dispose();
                _collider.Dispose();
            }
        }

        private readonly MainWindow _parent;
        private readonly Ground _ground;

        private static MeshMaterial _defaultMaterial = new MeshMaterial
        {
            Ambient = new OpenTK.Mathematics.Vector4(0.1f, 0.1f, 0.0f, 1.0f),
            Diffuse = new OpenTK.Mathematics.Vector4(0.8f, 0.8f, 0.0f, 1.0f),
            Specular = new OpenTK.Mathematics.Vector4(0.5f, 0.5f, 0.0f, 1.0f),
            Shininess = 8
        };

        public string[] Shapes { get; } = Enum.GetNames(typeof(ObstacleShape));

        public List<Obstacle> Obstacles { get; } = new();

        public int Count => Obstacles.Count;

        public ObstacleHandler(MainWindow parent)
        {
            _parent = parent;
        }

        public void Add(params Obstacle[] obstacles)
        {
            if (obstacles == null)
                throw new ArgumentNullException("obstacles");

            foreach (var obst in obstacles)
            {
                if (obst != null)
                    Obstacles.Add(obst);
            }
        }

        public void AddDefault(ObstacleShape shape)
        {
            switch (shape)
            {
                case ObstacleShape.Box:
                    Add(new Obstacle(Primitives.Cube(0.5f, 0.5f, 0.5f, _defaultMaterial),
                        _parent.PhysicsHandler.CreateCollider(RigidBodyType.Kinematic, new BoxShape(0.5f, 0.5f, 0.5f))));
                    break;
                case ObstacleShape.Sphere:
                    Add(new Obstacle(Primitives.Sphere(0.5f, 50, 50, _defaultMaterial),
                        _parent.PhysicsHandler.CreateCollider(RigidBodyType.Kinematic, new SphereShape(0.5f))));
                    break;
                case ObstacleShape.Cylinder:
                    Add(new Obstacle(new Model(new Mesh[]
                    {
                        Primitives.Cylinder(0.25f, 1f, 1f, 50, _defaultMaterial)
                    }), _parent.PhysicsHandler.CreateCollider(RigidBodyType.Kinematic, new CylinderShape(0.25f, 1f, 0.25f))));
                    break;
                case ObstacleShape.Cone:
                    Add(new Obstacle(new Model(new Mesh[]
                    {
                        Primitives.Cone(0.5f, 2, 50, _defaultMaterial)
                    }), _parent.PhysicsHandler.CreateCollider(RigidBodyType.Kinematic, new ConeShape(0.5f, 2))));  // TODO: cone's rigid body center is not the circle center, but the center of mass; fix
                    break;
                default:
                    throw new ArgumentException("The given obstacle shape is not implemented yet.", "shape");
            }
        }

        public void Remove(Obstacle obstacle)
        {
            if (Obstacles.Remove(obstacle))
                obstacle.Dispose();
        }

        // TODO: perhaps return all containing obstacles?
        public bool ContainmentTest(System.Numerics.Vector3 point, out Obstacle container)
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

        public void ToDesign()
        {
            foreach (var obst in Obstacles)
            {
                obst.Convert(RigidBodyType.Kinematic, obst.Mass);
                obst.Reset();
            }
        }

        public void ToAnimate()
        {
            foreach (var obst in Obstacles)
            {
                obst.Convert(obst.Type, obst.Mass);
            }
        }

        public void UpdateDesign()
        {
            foreach (var obst in Obstacles)
            {
                obst.UpdateStateDesign();
            }
        }

        public void UpdateAnimate()
        {
            foreach (var obst in Obstacles)
            {
                obst.UpdateStateAnimate();
            }
        }

        public void UpdateModel()
        {
            foreach (var obst in Obstacles)
            {
                obst.UpdateModel();

                obst.Path.Model.Update();
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
            foreach (var obst in Obstacles)
            {
                if (obst.Type == RigidBodyType.Kinematic)
                    obst.Path.Model.Render(shader);

                if (!obst.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    obst.Render(shader);
            }
        }

        public void RenderSelected(Shader shader)
        {
            foreach (var obst in Obstacles)
            {
                if (obst.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    obst.Render(shader);
            }
        }

        public void Dispose()
        {
            // dispose of the ground
            _ground.Dispose();

            // dispose of all the obstacles
            foreach (var obstacle in Obstacles)
            {
                obstacle.Dispose();
            }
        }
    }
}
