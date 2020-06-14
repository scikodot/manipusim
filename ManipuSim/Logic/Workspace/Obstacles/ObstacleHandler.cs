using System;
using System.Collections.Generic;

using OpenToolkit.Graphics.OpenGL4;
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

    public static class ObstacleHandler
    {
        private static class Ground
        {
            private readonly static Model _grid = Primitives.Grid(21, 1, MeshMaterial.White);

            private readonly static Model _ground = Primitives.Plane(10, 10, new MeshMaterial
            {
                Diffuse = new OpenToolkit.Mathematics.Vector4(1.0f, 1.0f, 1.0f, 0.5f)
            });

            private readonly static Collider _collider = PhysicsHandler.CreateStaticCollider(
                new BoxShape(10, 0.2f, 10),
                Matrix.Translation(-0.2f * BulletSharp.Math.Vector3.UnitY));

            public static void RenderGround(Shader shader)
            {
                _ground.Render(shader);
            }

            public static void RenderGrid(Shader shader)
            {
                _grid.Render(shader, () =>
                {
                    GL.DrawElements(BeginMode.Lines, _grid.Meshes[0].Indices.Length, DrawElementsType.UnsignedInt, 0);
                });
            }

            public static void Dispose()
            {
                // clear managed resources
                _grid.Dispose();
                _ground.Dispose();
                _collider.Dispose();
            }
        }

        private static MeshMaterial _defaultMaterial = new MeshMaterial
        {
            Ambient = new OpenToolkit.Mathematics.Vector4(0.1f, 0.1f, 0.0f, 1.0f),
            Diffuse = new OpenToolkit.Mathematics.Vector4(0.8f, 0.8f, 0.0f, 1.0f),
            Specular = new OpenToolkit.Mathematics.Vector4(0.5f, 0.5f, 0.0f, 1.0f),
            Shininess = 8
        };

        public static string[] Shapes { get; } = Enum.GetNames(typeof(ObstacleShape));

        public static List<Obstacle> Obstacles { get; } = new List<Obstacle>();

        public static int Count => Obstacles.Count;

        public static void Add(params Obstacle[] obstacles)
        {
            if (obstacles == null)
                throw new ArgumentNullException("obstacles");

            foreach (var obst in obstacles)
            {
                if (obst != null)
                    Obstacles.Add(obst);
            }
        }

        public static void AddDefault(ObstacleShape shape)
        {
            switch (shape)
            {
                case ObstacleShape.Box:
                    Add(new Obstacle(Primitives.Cube(0.5f, 0.5f, 0.5f, _defaultMaterial),
                        PhysicsHandler.CreateKinematicCollider(new BoxShape(0.5f, 0.5f, 0.5f))));
                    break;
                case ObstacleShape.Sphere:
                    Add(new Obstacle(Primitives.Sphere(0.5f, 50, 50, _defaultMaterial),
                        PhysicsHandler.CreateKinematicCollider(new SphereShape(0.5f))));
                    break;
                case ObstacleShape.Cylinder:
                    Add(new Obstacle(new Model(new Mesh[]
                    {
                        Primitives.Cylinder(0.25f, 1f, 1f, 50, _defaultMaterial)
                    }), PhysicsHandler.CreateKinematicCollider(new CylinderShape(0.25f, 1f, 0.25f))));
                    break;
                case ObstacleShape.Cone:
                    Add(new Obstacle(new Model(new Mesh[]
                    {
                        Primitives.Cone(0.5f, 2, 50, _defaultMaterial)
                    }), PhysicsHandler.CreateKinematicCollider(new ConeShape(0.5f, 2))));  // TODO: cone's rigid body center is not the circle center, but the center of mass; fix
                    break;
                default:
                    throw new ArgumentException("The given obstacle shape is not implemented yet.", "shape");
            }
        }

        public static void Remove(Obstacle obstacle)
        {
            if (Obstacles.Remove(obstacle))
                obstacle.Dispose();
        }

        // TODO: perhaps return all containing obstacles?
        public static bool ContainmentTest(System.Numerics.Vector3 point, out Obstacle container)
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

        public static void ToDesign()
        {
            foreach (var obst in Obstacles)
            {
                obst.Convert(RigidBodyType.Kinematic, obst.Mass);
                obst.Reset();
            }
        }

        public static void ToAnimate()
        {
            foreach (var obst in Obstacles)
            {
                obst.Convert(obst.Type, obst.Mass);
            }
        }

        public static void UpdateDesign()
        {
            foreach (var obst in Obstacles)
            {
                obst.UpdateStateDesign();
            }
        }

        public static void UpdateAnimate()
        {
            foreach (var obst in Obstacles)
            {
                obst.UpdateStateAnimate();
            }
        }

        public static void UpdateModel()
        {
            foreach (var obst in Obstacles)
            {
                obst.UpdateModel();

                obst.PathModel.Update(obst.Path);
            }
        }

        public static void RenderGrid(Shader shader)
        {
            Ground.RenderGrid(shader);
        }

        public static void RenderGround(Shader shader)
        {
            Ground.RenderGround(shader);
        }

        public static void RenderUnselected(Shader shader)
        {
            foreach (var obst in Obstacles)
            {
                if (obst.Type == RigidBodyType.Kinematic)
                    obst.PathModel.Render(shader);

                if (!obst.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    obst.Render(shader);
            }
        }

        public static void RenderSelected(Shader shader)
        {
            foreach (var obst in Obstacles)
            {
                if (obst.Model.RenderFlags.HasFlag(RenderFlags.Selected))
                    obst.Render(shader);
            }
        }

        public static void Dispose()
        {
            // dispose of the ground
            Ground.Dispose();

            // dispose of all the obstacles
            foreach (var obstacle in Obstacles)
            {
                obstacle.Dispose();
            }
        }
    }
}
