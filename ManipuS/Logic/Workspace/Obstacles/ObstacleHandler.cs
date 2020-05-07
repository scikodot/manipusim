using Graphics;
using Physics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Logic
{
    public static class ObstacleHandler
    {
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

        public static void DesignUpdate()
        {
            foreach (var obst in Obstacles)
            {
                obst.UpdateStateDesign();
            }
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

        public static void RenderAll(Shader shader)
        {
            foreach (var obst in Obstacles)
            {
                obst.Render(shader, MeshMode.Solid | MeshMode.Lighting);
            }
        }
    }
}
