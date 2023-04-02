using System;
using System.IO;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using StbImageSharp;

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

    public struct MeshTexture
    {
        // TODO: might cause concurrency issues;
        // _texturesLoaded is accessed from the aux loading thread,
        // while _texturesInitialized is accessed from the main context thread
        private static readonly Dictionary<string, ImageResult> _texturesLoaded = new();
        private static readonly Dictionary<string, int> _texturesInitialized = new();

        public int ID;
        public string Path;

        public void Load()
        {
            // Path is null -> nothing to load;
            // texture w/ Path is loaded or initialized -> no need to load again
            if (Path == null || _texturesLoaded.ContainsKey(Path) || _texturesInitialized.ContainsKey(Path))
                return;

            using var stream = new FileStream(Path, FileMode.Open);
            _texturesLoaded[Path] = ImageResult.FromStream(stream);
        }

        public void Initialize()
        {
            // default texture should also be considered initialized, 
            // hence use an empty string as its key
            Path ??= "";

            // texture w/ Path is initialized -> use its ID
            if (_texturesInitialized.ContainsKey(Path))
            {
                ID = _texturesInitialized[Path];
                return;
            }

            // no need to store texture image data anymore
            _texturesLoaded.Remove(Path, out var image);

            // if no texture image is supplied, generate a placeholder texture of 1 white pixel
            var data = image?.Data ?? new byte[] { 255, 255, 255, 255 };
            int width = image?.Width ?? 1;
            int height = image?.Height ?? 1;

            // TODO: this should be determined based on image.SourceComp, 
            // but PixelInternalFormat and PixelFormat seem incompatible on some values
            var format = PixelInternalFormat.Rgba;

            // load and generate the texture
            GL.GenTextures(1, out ID);
            
            // set texture data
            GL.BindTexture(TextureTarget.Texture2D, ID);
            GL.TexImage2D(TextureTarget.Texture2D, 0, format, width, height, 0, (PixelFormat)format, PixelType.UnsignedByte, data);
            GL.GenerateMipmap(GenerateMipmapTarget.Texture2D);

            // set texture parameters
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)TextureWrapMode.Repeat);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)TextureWrapMode.Repeat);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.LinearMipmapLinear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);

            _texturesInitialized[Path] = ID;
        }
    }

    public struct MeshMaterial  // [obsolete] TODO: consider using readonly refs
    {
        public MeshTexture TextureDiffuse = new();
        public MeshTexture TextureSpecular = new();
        public Color4 ColorAmbient = new(0, 0, 0, 0);
        public Color4 ColorDiffuse = Color4.Yellow;
        public Color4 ColorSpecular = new(0, 0, 0, 0);
        public float Shininess = 0;

        public MeshMaterial() { }
    }

    public class Mesh : IDisposable
    {
        private int VAO, VBO, EBO;

        public MeshVertex[] Vertices { get; }
        public uint[] Indices { get; }

        private MeshMaterial _material;
        public MeshMaterial Material => _material;
        public string Name { get; }
        public bool IsSetup { get; private set; }

        public Mesh(MeshVertex[] vertices, uint[] indices = null, MeshMaterial? material = null, string name = null)
        {
            Vertices = vertices;
            Indices = indices ?? Array.Empty<uint>();
            _material = material ?? new();
            Name = name;

            // load all the existent texture files
            _material.TextureDiffuse.Load();
            _material.TextureSpecular.Load();

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

                // setup textures
                _material.TextureDiffuse.Initialize();
                _material.TextureSpecular.Initialize();

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

        public void Render(ShaderProgram shader, PrimitiveType type, int? count, RenderFlags mode)
        {
            GL.BindVertexArray(VAO);

            // set colors
            shader.SetColor4("material.ambientCol", _material.ColorAmbient);
            shader.SetColor4("material.diffuseCol", _material.ColorDiffuse);
            shader.SetColor4("material.specularCol", _material.ColorSpecular);
            shader.SetFloat("material.shininess", _material.Shininess);

            // set textures
            // TODO: this is a stub that works only for meshes with no external textures;
            // replace with actual bindings of arbitrary textures
            GL.ActiveTexture(TextureUnit.Texture0);
            shader.SetInt("material.diffuseTex", 0);
            GL.BindTexture(TextureTarget.Texture2D, _material.TextureDiffuse.ID);

            GL.ActiveTexture(TextureUnit.Texture1);
            shader.SetInt("material.specularTex", 1);
            GL.BindTexture(TextureTarget.Texture2D, _material.TextureSpecular.ID);

            GL.ActiveTexture(TextureUnit.Texture0);

            // set different rendering options
            // TODO: use the same words as for RenderFlags
            shader.SetBool("enableWireframe", (uint)(mode & RenderFlags.Wireframe));
            shader.SetBool("enableLighting", (uint)(mode & RenderFlags.Lighting));
            shader.SetBool("isSelected", (uint)(mode & RenderFlags.Selected));

            // render wireframe
            // TODO: replace with single-pass render, i.e. through geometry shader or anything
            if (mode.HasFlag(RenderFlags.Wireframe))  
            {
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
                GL.PointSize(2);
                GL.DrawElements(BeginMode.Points, Indices.Length, DrawElementsType.UnsignedInt, 0);
                GL.PointSize(1);
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            }

            // render mesh
            if (Indices.Length != 0)
                GL.DrawElements(type, count ?? Indices.Length, DrawElementsType.UnsignedInt, 0);
            else
                GL.DrawArrays(type, 0, count ?? Vertices.Length);

            GL.BindVertexArray(0);
        }

        public Mesh Copy() => new(Vertices, Indices, Material, Name);

        public void Dispose()
        {
            // TODO: delete textures?
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
