using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK;
using OpenTK.Graphics.OpenGL4;
using System.Runtime.InteropServices;

namespace Graphics
{
    struct MeshVertex
    {
        public Vector3 Position;
        public Vector3 Normal;
        public Vector2 TexCoords;
    }

    struct MeshTexture
    {
        public int ID;
        public string Type;
        public string Path;
    }

    class Mesh
    {
        public MeshVertex[] Vertices;
        public int[] Indices;
        public MeshTexture[] Textures;

        private int VAO, VBO, EBO;

        public Mesh(MeshVertex[] vertices, int[] indices, MeshTexture[] textures)
        {
            Vertices = vertices;
            Indices = indices;
            Textures = textures;

            SetupMesh();
        }

        private void SetupMesh()
        {
            VAO = GL.GenVertexArray();
            VBO = GL.GenBuffer();
            EBO = GL.GenBuffer();

            GL.BindVertexArray(VAO);

            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, Vertices.Length * Marshal.SizeOf(typeof(MeshVertex)), Vertices, BufferUsageHint.StaticDraw);

            GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
            GL.BufferData(BufferTarget.ElementArrayBuffer, Indices.Length * sizeof(uint), Indices, BufferUsageHint.StaticDraw);

            // vertex positions
            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, Marshal.SizeOf(typeof(MeshVertex)), 0);

            // vertex normals
            GL.EnableVertexAttribArray(1);
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, Marshal.SizeOf(typeof(MeshVertex)), Marshal.OffsetOf<MeshVertex>("Normal"));

            // vertex texture coords
            GL.EnableVertexAttribArray(2);
            GL.VertexAttribPointer(2, 2, VertexAttribPointerType.Float, false, Marshal.SizeOf(typeof(MeshVertex)), Marshal.OffsetOf<MeshVertex>("TexCoords"));

            GL.BindVertexArray(0);
        }

        public void Draw(Shader shader)
        {
            int diffuseNr = 1;
            int specularNr = 1;
            for (int i = 0; i < Textures.Length; i++)
            {
                GL.ActiveTexture(TextureUnit.Texture0 + i);  //activate proper texture unit before binding

                // retrieve texture number
                string number = "";
                string name = Textures[i].Type;
                if (name == "texture_diffuse")
                    number = diffuseNr++.ToString();
                else if (name == "texture_specular")
                    number = specularNr++.ToString();
                
                shader.SetInt(name + number, i);  // TODO: concat with "material." if a material struct is defined in shader program
                GL.BindTexture(TextureTarget.Texture2D, Textures[i].ID);
            }
            GL.ActiveTexture(TextureUnit.Texture0);

            // draw mesh
            GL.BindVertexArray(VAO);
            GL.DrawElements(BeginMode.Triangles, Indices.Length, DrawElementsType.UnsignedInt, 0);
            GL.BindVertexArray(0);
        }
    }
}
