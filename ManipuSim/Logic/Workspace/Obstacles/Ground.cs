using BulletSharp;
using Graphics;
using OpenTK.Graphics.OpenGL;
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
        public Model GridModel { get; } = Primitives.Grid(21, 1, MeshMaterial.White);
        public Model GroundModel { get; } = Primitives.Plane(10, 10, new MeshMaterial
        {
            Diffuse = new OpenTK.Mathematics.Vector4(1.0f, 1.0f, 1.0f, 0.5f)
        });
        public Collider Collider { get; }

        public Ground(Collider collider)
        {
            Collider = collider;
        }

        public void RenderGround(Shader shader)
        {
            GroundModel.Render(shader);
        }

        public void RenderGrid(Shader shader)
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
