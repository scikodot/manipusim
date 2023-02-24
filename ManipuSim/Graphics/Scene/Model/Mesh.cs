using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Runtime.InteropServices;

using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;

namespace Graphics
{
    public struct MeshVertex
    {
        public static int Size => Marshal.SizeOf<MeshVertex>();

        public Vector3 Position;
        public Vector3 Normal;
        public Vector2 TexCoords;

        public static MeshVertex[] Convert(IEnumerable<System.Numerics.Vector3> vertices)
        {
            return vertices.Select(v => new MeshVertex
            {
                Position = new Vector3(v.X, v.Y, v.Z)
            }).ToArray();
        }
    }

    // reference has to be hold for texture generation in GL.GenTextures() in main thread => class instead of struct;
    // see TextureFromFile() in Model class
    public class MeshTexture
    {
        public int ID;
        public string Type;
        public string Path;
    }

    public struct MeshMaterial  // TODO: consider using readonly refs
    {
        public static MeshMaterial Black => new MeshMaterial { Diffuse = Vector4.UnitW };
        public static MeshMaterial Red => new MeshMaterial { Diffuse = new Vector4(1.0f, 0.0f, 0.0f, 1.0f) };
        public static MeshMaterial Green => new MeshMaterial { Diffuse = new Vector4(0.0f, 1.0f, 0.0f, 1.0f) };
        public static MeshMaterial Blue => new MeshMaterial { Diffuse = new Vector4(0.0f, 0.0f, 1.0f, 1.0f) };
        public static MeshMaterial Yellow => new MeshMaterial { Diffuse = new Vector4(1.0f, 1.0f, 0.0f, 1.0f) };
        public static MeshMaterial Pink => new MeshMaterial { Diffuse = new Vector4(1.0f, 0.0f, 1.0f, 1.0f) };
        public static MeshMaterial Cyan => new MeshMaterial { Diffuse = new Vector4(0.0f, 1.0f, 1.0f, 1.0f) };
        public static MeshMaterial White => new MeshMaterial { Diffuse = Vector4.One };

        public static MeshMaterial Brown => new MeshMaterial { Diffuse = new Vector4(0.6f, 0.2f, 0.08f, 1.0f) };

        public Vector4 Ambient;
        public Vector4 Diffuse;
        public Vector4 Specular;
        public float Shininess;
    }

    public class Mesh : IDisposable
    {
        private int VAO, VBO, EBO;
        public bool IsSetup { get; private set; }

        public string Name { get; }

        public MeshVertex[] Vertices { get; }
        public uint[] Indices { get; }
        public MeshTexture[] Textures { get; }
        public MeshMaterial Material { get; set; }

        public Mesh(string name, MeshVertex[] vertices, uint[] indices, MeshTexture[] textures, MeshMaterial material)
        {
            Name = name;
            Vertices = vertices;
            Indices = indices;
            Textures = textures;
            Material = material;

            // setup can be done only on the main thread, holding the GL context;
            // hence, send necessary actions to dispatcher
            Dispatcher.RenderActions.Enqueue(() =>
            {
                SetupMesh();
            });

            /*
            if (Thread.CurrentThread == MainWindow.MainThread)
                SetupMesh();
            else
            {
                
            }*/
        }

        private void SetupMesh()
        {
            VAO = GL.GenVertexArray();
            VBO = GL.GenBuffer();
            EBO = GL.GenBuffer();

            GL.BindVertexArray(VAO);

            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, Vertices.Length * MeshVertex.Size, Vertices, BufferUsageHint.StaticDraw);

            GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
            GL.BufferData(BufferTarget.ElementArrayBuffer, Indices.Length * sizeof(uint), Indices, BufferUsageHint.StaticDraw);

            // vertex positions
            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, MeshVertex.Size, 0);

            // vertex normals
            GL.EnableVertexAttribArray(1);
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, MeshVertex.Size, Marshal.OffsetOf<MeshVertex>("Normal"));

            // vertex texture coords
            GL.EnableVertexAttribArray(3);
            GL.VertexAttribPointer(3, 2, VertexAttribPointerType.Float, false, MeshVertex.Size, Marshal.OffsetOf<MeshVertex>("TexCoords"));

            GL.BindVertexArray(0);

            IsSetup = true;
        }

        public void UpdateVertices(uint offset, int size, MeshVertex[] vertices)
        {
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferSubData(BufferTarget.ArrayBuffer, (IntPtr)(offset * MeshVertex.Size), size * MeshVertex.Size, vertices);
        }

        public void UpdateIndices(uint offset, int size, uint[] indices)
        {
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
            GL.BufferSubData(BufferTarget.ElementArrayBuffer, (IntPtr)(offset * sizeof(uint)), size * sizeof(uint), indices);
        }

        public void Render(Shader shader, RenderFlags mode, Action render)
        {
            GL.BindVertexArray(VAO);

            if (mode.HasFlag(RenderFlags.Solid))
            {
                shader.SetBool("enableWireframe", 0);
                shader.SetBool("enableLighting", mode.HasFlag(RenderFlags.Lighting) ? 1u : 0u);
                shader.SetBool("enableTextures", Textures.Length == 0 ? 0u : 1u);
                shader.SetBool("isSelected", mode.HasFlag(RenderFlags.Selected) ? 1u : 0u);

                // set textures
                int diffuseNr = 1;
                int specularNr = 1;
                for (int i = 0; i < Textures.Length; i++)
                {
                    //activate proper texture unit before binding
                    GL.ActiveTexture(TextureUnit.Texture0 + i);

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
                shader.SetVector4("material.ambientCol", Material.Ambient);  // TODO: add ref
                shader.SetVector4("material.diffuseCol", Material.Diffuse);
                shader.SetVector4("material.specularCol", Material.Specular);
                shader.SetFloat("material.shininess", Material.Shininess);

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
                        GL.DrawArrays(PrimitiveType.Triangles, 0, Vertices.Length);
                }
            }

            if (mode.HasFlag(RenderFlags.Wireframe))  // TODO: replace with single-pass render, i.e. through geometry shader or anything
            {
                shader.SetBool("enableWireframe", 1);  // TODO: use the same words as for RenderFlags

                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
                GL.PointSize(2);
                GL.DrawElements(BeginMode.Points, Indices.Length, DrawElementsType.UnsignedInt, 0);
                GL.PointSize(1);
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            }

            if (mode == 0)
            {
                // TODO: maybe notify the user about drawing nothing?
            }

            GL.BindVertexArray(0);
        }

        public Mesh DeepCopy()
        {
            return new Mesh(Name, Vertices, Indices, Textures, Material);
        }

        public void Dispose()
        {
            // dispose the mesh
            DisposeInner();

            // suppress additional finalization, because all unmanaged resources have already been closed
            GC.SuppressFinalize(this);
        }

        protected virtual void DisposeInner()
        {
            // TODO: check for disposed; see documentation

            // clear unmanaged resources
            Dispatcher.RenderActions.Enqueue(() =>
            {
                GL.DeleteVertexArray(VAO);
                GL.DeleteBuffer(VBO);
                GL.DeleteBuffer(EBO);
            });

            Console.WriteLine($"Disposed mesh: VAO - {VAO}, VBO - {VBO}");
        }

        // TODO: finalizer runs AFTER the GL context is already deleted, 
        // so all buffers have to be freed manually on context destruction!
        //~Mesh()
        //{
        //    // clear all resources if they haven't been cleared by the user
        //    DisposeInner();
        //}
    }
}
