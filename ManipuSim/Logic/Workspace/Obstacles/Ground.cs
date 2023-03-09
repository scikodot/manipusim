using BulletSharp;
using Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using Physics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Logic
{
    public class Ground : IDisposable
    {
        private readonly static MeshMaterial _defaultGroundMaterial = new() { Diffuse = new(1.0f, 1.0f, 1.0f, 0.5f) };
        private readonly static MeshMaterial _defaultGridMaterial = new() { Diffuse = Color4.White };

        public Model GroundModel { get; }
        public Model GridModel { get; }
        public Collider Collider { get; }

        public Ground(Collider collider)
        {
            GridModel = new Model(new Mesh[]
            {
                Primitives.Grid(21, 1, _defaultGridMaterial)
            });

            GroundModel = new Model(new Mesh[]
            {
                Primitives.Plane(10, 10, _defaultGroundMaterial)
            });

            Collider = collider;
        }

        public void RenderGround(ShaderProgram shader)
        {
            GroundModel.Render(shader);
        }

        public void RenderGrid(ShaderProgram shader)
        {
            GridModel.Render(shader, () =>
            {
                GL.DrawElements(BeginMode.Lines, GridModel.Meshes[0].Indices.Length, DrawElementsType.UnsignedInt, 0);
            });
        }

        public void Dispose()
        {
            // clear managed resources
            GridModel.Dispose();
            GroundModel.Dispose();
            Collider.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
