using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Assimp;
using OpenTK;
using OpenTK.Graphics.OpenGL4;
using System.Runtime.InteropServices;
using System.Threading;

namespace Graphics
{
    public struct MeshVertex
    {
        public Vector3 Position;
        public Vector3 Normal;
        public Vector2 TexCoords;

        public static MeshVertex[] Convert(float[] vertices)
        {
            var vertNum = vertices.Length / 6;
            var meshVertices = new MeshVertex[vertNum];
            for (int i = 0; i < vertNum; i++)
            {
                meshVertices[i] = new MeshVertex
                {
                    Position = new Vector3(vertices[6 * i], vertices[6 * i + 1], vertices[6 * i + 2]),
                    Normal = new Vector3(vertices[6 * i + 3], vertices[6 * i + 4], vertices[6 * i + 5])
                };
            }

            return meshVertices;
        }

        public static MeshVertex[] Convert(System.Numerics.Vector3[] vertices)
        {
            return vertices.Select(v => new MeshVertex
            {
                Position = new Vector3(v.X, v.Y, v.Z)
            }).ToArray();
        }
    }

    public struct MeshTexture
    {
        public int ID;
        public string Type;
        public string Path;
    }

    public struct MeshMaterial
    {
        public static MeshMaterial Black => new MeshMaterial { Diffuse = Vector4.UnitW };
        public static MeshMaterial Red => new MeshMaterial { Diffuse = new Vector4(1.0f, 0.0f, 0.0f, 1.0f) };
        public static MeshMaterial Green => new MeshMaterial { Diffuse = new Vector4(0.0f, 1.0f, 0.0f, 1.0f) };
        public static MeshMaterial Blue => new MeshMaterial { Diffuse = new Vector4(0.0f, 0.0f, 1.0f, 1.0f) };
        public static MeshMaterial Yellow => new MeshMaterial { Diffuse = new Vector4(1.0f, 1.0f, 0.0f, 1.0f) };
        public static MeshMaterial Pink => new MeshMaterial { Diffuse = new Vector4(1.0f, 0.0f, 1.0f, 1.0f) };
        public static MeshMaterial Cyan => new MeshMaterial { Diffuse = new Vector4(0.0f, 1.0f, 1.0f, 1.0f) };
        public static MeshMaterial White => new MeshMaterial { Diffuse = Vector4.One };

        public Vector4 Ambient;
        public Vector4 Diffuse;
        public Vector4 Specular;
        public float Shininess;
    }

    [Flags]
    public enum MeshMode
    {
        Solid = 1,
        Wireframe = 2,
        Lighting = 4
    }

    public class Mesh
    {
        public string Name;

        public MeshVertex[] Vertices;
        public uint[] Indices;
        public MeshTexture[] Textures;
        public MeshMaterial Color;

        public Vector3 Position;

        private int VAO, VBO, EBO;

        public Mesh(string name, MeshVertex[] vertices, uint[] indices, Material material)
        {

        }

        public Mesh(string name, MeshVertex[] vertices, uint[] indices, MeshTexture[] textures, MeshMaterial color)
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

            // setup can be done only on the main thread, holding the GL context
            if (Thread.CurrentThread == MainWindow.MainThread)
                SetupMesh();
            else
            {
                // send necessary actions to dispatcher
                Dispatcher.RenderActions.Enqueue(() =>
                {
                    SetupMesh();
                });
            }
        }

        private void SetupMesh()
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
        }

        //public void Update(float[] vertices, uint[] indices)
        //{
        //    var verts = MeshVertex.Convert(vertices);
        //    GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
        //    GL.BufferData(BufferTarget.ArrayBuffer, verts.Length * Marshal.SizeOf<MeshVertex>(), verts, BufferUsageHint.StaticDraw);

        //    GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
        //    GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(uint), indices, BufferUsageHint.StaticDraw);
        //}

        public void Update(MeshVertex[] vertices, uint[] indices)
        {
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            //GL.InvalidateBufferData(VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * Marshal.SizeOf<MeshVertex>(), vertices, BufferUsageHint.DynamicDraw);

            GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
            //GL.InvalidateBufferData(EBO);
            GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(uint), indices, BufferUsageHint.DynamicDraw);
        }

        public void Render(Shader shader, MeshMode mode, Action render)
        {
            GL.BindVertexArray(VAO);

            if ((mode & MeshMode.Wireframe) == MeshMode.Wireframe)
            {
                shader.SetBool("enableWireframe", 1);

                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
                GL.PointSize(2);
                GL.DrawElements(BeginMode.Points, Indices.Length, DrawElementsType.UnsignedInt, 0);
                GL.PointSize(1);
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            }

            if ((mode & MeshMode.Solid) == MeshMode.Solid)
            {
                shader.SetBool("enableWireframe", 0);

                if ((mode & MeshMode.Lighting) == MeshMode.Lighting)
                {
                    shader.SetBool("enableLighting", 1);
                }
                else
                {
                    shader.SetBool("enableLighting", 0);
                }

                shader.SetBool("enableTextures", Textures.Length == 0 ? 0u : 1u);

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
                shader.SetVector4("material.ambientCol", Color.Ambient/*new Vector4(Color.Ambient.R, Color.Ambient.G, Color.Ambient.B, Color.Ambient.A)*/);
                shader.SetVector4("material.diffuseCol", Color.Diffuse/*new Vector4(Color.Diffuse.R, Color.Diffuse.G, Color.Diffuse.B, Color.Diffuse.A)*/);
                shader.SetVector4("material.specularCol", Color.Specular/*new Vector4(Color.Specular.R, Color.Specular.G, Color.Specular.B, Color.Specular.A)*/);
                shader.SetFloat("material.shininess", Color.Shininess);

                // render mesh
                if (render != null)
                {
                    render();
                }
                else
                {
                    // if render action is not specified, use default
                    if (Indices.Length != 0)
                        GL.DrawElements(BeginMode.Triangles, Indices.Length, DrawElementsType.UnsignedInt, 0);
                    else
                        GL.DrawArrays(OpenTK.Graphics.OpenGL4.PrimitiveType.Triangles, 0, Vertices.Length);
                }
            }

            if (mode == 0)
            {
                // TODO: maybe notify the user about drawing nothing?
            }

            GL.BindVertexArray(0);
        }

        public void Dispose(bool disposedByUser)
        {
            // TODO: check for disposed; see documentation

            if (disposedByUser)
            {
                // clear managed resources
                Vertices = null;
                Indices = null;
                Textures = null;
            }

            // clear unmanaged resources
            GL.DeleteBuffer(EBO);
            GL.DeleteBuffer(VBO);
            GL.DeleteVertexArray(VAO);

            Console.WriteLine($"Disposed model: VAO - {VAO}, VBO - {VBO}");
        }
    }
}
