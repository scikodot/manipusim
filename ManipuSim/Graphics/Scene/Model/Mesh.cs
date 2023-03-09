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
        public static readonly int Size = Marshal.SizeOf<MeshVertex>();
        public static readonly IntPtr NormalOffset = Marshal.OffsetOf<MeshVertex>("Normal");
        public static readonly IntPtr TexCoordsOffset = Marshal.OffsetOf<MeshVertex>("TexCoords");

        public Vector3 Position;
        public Vector3 Normal;
        public Vector2 TexCoords;

        /*public static MeshVertex[] Convert(IEnumerable<System.Numerics.Vector3> vertices)
        {
            return vertices.Select(v => new MeshVertex
            {
                Position = new Vector3(v.X, v.Y, v.Z)
            }).ToArray();
        }*/
    }

    // TODO: revisit
    // reference has to be hold for texture generation in GL.GenTextures() in main thread => class instead of struct;
    // see TextureFromFile() in Model class
    public class MeshTexture
    {
        public int ID;
        public string Type;
        public string Path;
    }

    // TODO: consider injecting MeshTexture into MeshMaterial, or even replacing both with Assimp.Material
    // (though might not be a great idea, as that class is used primarily for imports); see Assimp.Material class 
    public struct MeshMaterial  // [obsolete] TODO: consider using readonly refs
    {
        public Color4 Ambient;
        public Color4 Diffuse;
        public Color4 Specular;
        public float Shininess;
    }

    public class Mesh : IDisposable
    {
        private readonly static MeshMaterial _defaultMaterial = new() { Diffuse = Color4.Yellow };

        private int VAO, VBO, EBO;

        public MeshVertex[] Vertices { get; }
        public uint[] Indices { get; }
        public MeshTexture[] Textures { get; }
        public MeshMaterial Material { get; set; }
        public string Name { get; }
        public bool IsSetup { get; private set; }

        public Mesh(MeshVertex[] vertices, uint[] indices = null, MeshTexture[] textures = null, 
            MeshMaterial? material = null, string name = null)
        {
            Vertices = vertices;
            Indices = indices ?? Array.Empty<uint>();
            Textures = textures ?? new[] { new MeshTexture() { Type = "diffuseTex" } };  /*Array.Empty<MeshTexture>();*/
            Material = material ?? _defaultMaterial;
            Name = name;

            // TODO: indices buffer (EBO) is bound for every Mesh, even for those that do not use indices,
            // which seems redundant

            // setup can only be done on the main thread, holding the GL context;
            // hence, send necessary actions to dispatcher
            Dispatcher.RenderActions.Enqueue(() =>
            {
                // create the array and bind it
                VAO = GL.GenVertexArray();
                GL.BindVertexArray(VAO);

                // create the vertex buffer and bind it to the array
                VBO = GL.GenBuffer();
                GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
                GL.BufferData(BufferTarget.ArrayBuffer, Vertices.Length * MeshVertex.Size, Vertices, BufferUsageHint.StaticDraw);

                // create the indices buffer and bind it to the array
                EBO = GL.GenBuffer();
                GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
                GL.BufferData(BufferTarget.ElementArrayBuffer, Indices.Length * sizeof(uint), Indices, BufferUsageHint.StaticDraw);

                // vertex positions
                GL.EnableVertexAttribArray(0);
                GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, MeshVertex.Size, 0);

                // vertex normals
                GL.EnableVertexAttribArray(1);
                GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, MeshVertex.Size, MeshVertex.NormalOffset);

                // vertex colors; we use materials though, so this one is not used
                // GL.EnableVertexAttribArray(2);
                // GL.VertexAttribPointer(2, 4, ...);

                // vertex texture coords
                GL.EnableVertexAttribArray(3);
                GL.VertexAttribPointer(3, 2, VertexAttribPointerType.Float, false, MeshVertex.Size, MeshVertex.TexCoordsOffset);

                // unbind the array
                GL.BindVertexArray(0);

                // for the compliance with the shader, if no textures are supplied,
                // generate a "placeholder" texture of 1 white pixel
                if (textures == null)
                {
                    GL.GenTextures(1, out Textures[0].ID);
                    GL.BindTexture(TextureTarget.Texture2D, Textures[0].ID);
                    GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba, 1, 1, 0, PixelFormat.Rgba, PixelType.UnsignedByte,
                        new byte[] { 255, 255, 255, 255 });
                }

                IsSetup = true;
            });
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

        public void Render(ShaderProgram shader, RenderFlags mode, Action render)
        {
            GL.BindVertexArray(VAO);

            if (mode.HasFlag(RenderFlags.Solid))
            {
                shader.SetBool("enableWireframe", 0);
                shader.SetBool("enableLighting", mode.HasFlag(RenderFlags.Lighting) ? 1u : 0u);
                shader.SetBool("isSelected", mode.HasFlag(RenderFlags.Selected) ? 1u : 0u);

                // TODO: below is a placeholder that works only for meshes with no external textures;
                // replace with actual bindings of arbitrary textures

                // set textures
                GL.ActiveTexture(TextureUnit.Texture0);
                shader.SetInt("material.diffuseTex", 0);
                GL.BindTexture(TextureTarget.Texture2D, Textures[0].ID);

                GL.ActiveTexture(TextureUnit.Texture1);
                shader.SetInt("material.specularTex", 1);
                GL.BindTexture(TextureTarget.Texture2D, Textures[0].ID);

                GL.ActiveTexture(TextureUnit.Texture0);

                // set colors
                shader.SetColor4("material.ambientCol", Material.Ambient);
                shader.SetColor4("material.diffuseCol", Material.Diffuse);
                shader.SetColor4("material.specularCol", Material.Specular);
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

        public Mesh DeepCopy() => new(Vertices, Indices, Textures, Material, Name);

        public void Dispose()
        {
            Dispatcher.RenderActions.Enqueue(() =>
            {
                GL.DeleteVertexArray(VAO);
                GL.DeleteBuffer(VBO);
                GL.DeleteBuffer(EBO);
            });

            // TODO: this has to be displayed immediately after the disposal actions (see above) are performed,
            // not after the Dispatcher receives a request for those actions
            // TODO: all logging should be centralized, say, in a Logger class
            Console.WriteLine($"Disposed mesh: VAO - {VAO}, VBO - {VBO}");

            GC.SuppressFinalize(this);
        }
    }
}
