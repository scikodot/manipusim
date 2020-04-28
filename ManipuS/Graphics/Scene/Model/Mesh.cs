using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Assimp;
using OpenTK;
using OpenTK.Graphics.OpenGL4;
using System.Runtime.InteropServices;

namespace Graphics
{
    struct MeshVertex  // TODO: try to use Assimp's data structures
    {
        public Vector3 Position;
        public Vector3 Normal;
        public Vector2 TexCoords;
    }

    // MeshTexture has to be class; otherwise it could not be passed by reference  // TODO: actually, it can be passed; fix!
    class MeshTexture  // TODO: try to use Assimp's data structures
    {
        public int ID;
        public string Type;
        public string Path;
    }

    struct MeshColor  // TODO: try to use Assimp's data structures
    {
        public Color4D Ambient;
        public Color4D Diffuse;
        public Color4D Specular;
        public float Shininess;
    }

    [Flags]
    public enum MeshMode
    {
        Solid = 1,
        Wireframe = 2
    }

    class Mesh
    {
        public string Name;

        public MeshVertex[] Vertices;
        public uint[] Indices;
        public MeshTexture[] Textures;
        public MeshColor Color;

        public Vector3 Position;

        private int VAO, VBO, EBO;

        public Mesh(string name, MeshVertex[] vertices, uint[] indices, MeshTexture[] textures, MeshColor color)
        {
            Name = name;
            Vertices = vertices;
            Indices = indices;
            Textures = textures;
            Color = color;

            Position = new Vector3
            {
                X = vertices.Sum(vertex => vertex.Position.X),
                Y = vertices.Sum(vertex => vertex.Position.Y),
                Z = vertices.Sum(vertex => vertex.Position.Z)
            };

            SetupMesh();
        }

        private void SetupMesh()
        {
            // sending necessary actions to dispatcher
            Dispatcher.RenderActions.Enqueue(() =>
            {
                VAO = GL.GenVertexArray();
                VBO = GL.GenBuffer();
                EBO = GL.GenBuffer();

                GL.BindVertexArray(VAO);

                GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
                GL.BufferData(BufferTarget.ArrayBuffer, Vertices.Length * Marshal.SizeOf<MeshVertex>(), Vertices, BufferUsageHint.StaticDraw);

                GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
                GL.BufferData(BufferTarget.ElementArrayBuffer, Indices.Length * sizeof(uint), Indices, BufferUsageHint.StaticDraw);

                // vertex positions
                GL.EnableVertexAttribArray(0);
                GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, Marshal.SizeOf<MeshVertex>(), 0);

                // vertex normals
                GL.EnableVertexAttribArray(1);
                GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, Marshal.SizeOf<MeshVertex>(), Marshal.OffsetOf<MeshVertex>("Normal"));

                // vertex texture coords
                GL.EnableVertexAttribArray(3);
                GL.VertexAttribPointer(3, 2, VertexAttribPointerType.Float, false, Marshal.SizeOf<MeshVertex>(), Marshal.OffsetOf<MeshVertex>("TexCoords"));

                GL.BindVertexArray(0);
            });
        }

        public void Render(Shader shader, MeshMode mode)
        {
            GL.BindVertexArray(VAO);

            if ((mode & MeshMode.Wireframe) == MeshMode.Wireframe)
            {
                shader.SetBool("useMaterial", 0);

                GL.PointSize(2);
                GL.DrawElements(BeginMode.Points, Indices.Length, DrawElementsType.UnsignedInt, 0);
                GL.PointSize(1);
            }

            if ((mode & MeshMode.Solid) == MeshMode.Solid)
            {
                shader.SetBool("useMaterial", 1);
                shader.SetBool("useTextures", Textures.Length == 0 ? 0u : 1u);

                // set textures
                int diffuseNr = 1;
                int specularNr = 1;
                for (int i = 0; i < Textures.Length; i++)
                {
                    GL.ActiveTexture(TextureUnit.Texture0 + i);  //activate proper texture unit before binding

                    // retrieve texture number
                    string number = "";
                    string name = Textures[i].Type;
                    if (name == "texture_diffuse")  // TODO: fix names in shaders
                        number = diffuseNr++.ToString();
                    else if (name == "texture_specular")
                        number = specularNr++.ToString();

                    shader.SetInt("material." + name + number, i);
                    GL.BindTexture(TextureTarget.Texture2D, Textures[i].ID);
                }
                GL.ActiveTexture(TextureUnit.Texture0);

                // set colors
                shader.SetVector3("material.ambientCol", new Vector3(Color.Ambient.R, Color.Ambient.G, Color.Ambient.B));
                shader.SetVector3("material.diffuseCol", new Vector3(Color.Diffuse.R, Color.Diffuse.G, Color.Diffuse.B));
                shader.SetVector3("material.specularCol", new Vector3(Color.Specular.R, Color.Specular.G, Color.Specular.B));
                shader.SetFloat("material.shininess", Color.Shininess);

                // render mesh
                GL.DrawElements(BeginMode.Triangles, Indices.Length, DrawElementsType.UnsignedInt, 0);
            }

            if (mode == 0)
            {
                // TODO: maybe notify the user about drawing nothing?
            }

            GL.BindVertexArray(0);
        }
    }
}
