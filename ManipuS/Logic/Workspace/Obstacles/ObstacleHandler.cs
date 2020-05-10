using System;
using System.Collections.Generic;

using BulletSharp;
using OpenTK;

using Graphics;
using Physics;
using BulletSharp.Math;

using Vector3 = OpenTK.Vector3;
using Vector4 = OpenTK.Vector4;
using OpenTK.Graphics.OpenGL4;

namespace Logic
{
    public static class ObstacleHandler
    {
        private static class Ground
        {
            private readonly static Model _grid = Primitives.Grid(21, 1, MeshMaterial.White);

            private readonly static Model _ground = Primitives.Plane(10, 10, new MeshMaterial
            {
                Diffuse = new Vector4(1.0f, 1.0f, 1.0f, 0.5f)
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

        public static string[] ObstacleShapes = new string[]
        {
            "Box",
            "Sphere",
            "Cylinder"
        };

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

        public static void Remove(Obstacle obstacle)
        {
            if (Obstacles.Remove(obstacle))
                obstacle.Dispose();
        }

        public static void ToDesign()
        {
            foreach (var obst in Obstacles)
            {
                obst.Convert(RigidBodyType.Kinematic);

                obst.Reset();
            }
        }

        public static void ToAnimate()
        {
            foreach (var obst in Obstacles)
            {
                obst.Convert(obst.Type);
            }
        }

        public static void UpdateDesign()
        {
            foreach (var obst in Obstacles)
            {
                obst.UpdateStateDesign();
            }
        }

        public static void UpdateModel()
        {
            foreach (var obst in Obstacles)
            {
                obst.UpdateModel();
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
