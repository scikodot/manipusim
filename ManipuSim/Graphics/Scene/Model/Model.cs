using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;

using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;
using Assimp;
using StbImageSharp;

using PrimitiveType = OpenTK.Graphics.OpenGL4.PrimitiveType;

namespace Graphics
{
    [Flags]
    public enum RenderFlags
    {
        Default = 0,
        Wireframe = 1,
        Lighting = 2,
        Selected = 4
    }

    public class Model : IDisposable
    {
        private static readonly Dictionary<string, MeshTexture> _texturesLoaded = new();

        public List<Mesh> Meshes { get; } = new();

        private Matrix4 _state;
        public Matrix4 State => _state;

        public RenderFlags RenderFlags { get; set; } = RenderFlags.Default;
        public bool IsSetup => Meshes.All(mesh => mesh.IsSetup);

        public Model(MeshVertex[] vertices, uint[] indices = null, MeshMaterial? material = null, Matrix4? state = null, 
            string name = null, RenderFlags renderFlags = RenderFlags.Default)
        {
            Meshes.Add(new Mesh(vertices, indices, material: material, name: name));

            _state = state ?? Matrix4.Identity;

            RenderFlags = renderFlags;
        }

        public Model(IEnumerable<Mesh> meshes, Matrix4? state = null, RenderFlags renderFlags = RenderFlags.Default)
        {
            Meshes.AddRange(meshes);

            _state = state ?? Matrix4.Identity;

            RenderFlags = renderFlags;
        }

        public Model(string path)
        {
            Load(path);
        }

        private void Load(string path)
        {
            using var importer = new AssimpContext();
            var scene = importer.ImportFile(path, PostProcessSteps.Triangulate | PostProcessSteps.FlipUVs);
            if (scene?.RootNode == null || scene.SceneFlags.HasFlag(SceneFlags.Incomplete))
            {
                // TODO: throw exception in case of an unsuccessful scene import
                return;
            }

            ProcessNode(Path.GetDirectoryName(path), scene, scene.RootNode);
        }

        private void ProcessNode(string directory, Scene scene, Node node)
        {
            // process all node's meshes, if any
            foreach (var index in node.MeshIndices)
                Meshes.Add(ProcessMesh(directory, scene, scene.Meshes[index]));

            // process all descendant nodes
            foreach (var child in node.Children)
                ProcessNode(directory, scene, child);
        }

        private static Mesh ProcessMesh(string directory, Scene scene, Assimp.Mesh mesh)
        {
            // process vertices
            var vertices = new List<MeshVertex>();
            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var vertex = new MeshVertex
                {
                    Position = mesh.Vertices[i].ToOpenTK(),
                    Normal = mesh.Normals[i].ToOpenTK()
                };

                // add texture coords if present
                if (mesh.HasTextureCoords(0))
                    vertex.TexCoords = mesh.TextureCoordinateChannels[0][i].ToOpenTK().Xy;

                vertices.Add(vertex);
            }

            // process indices
            var indices = mesh.Faces.SelectMany(face => face.Indices).Select(x => (uint)x);

            // process material
            var textures = new List<MeshTexture>();
            MeshMaterial? material = null;
            if (mesh.MaterialIndex >= 0)
            {
                Material mat = scene.Materials[mesh.MaterialIndex];

                // get material textures
                if (mat.HasTextureDiffuse)
                {
                    var diffuseMaps = LoadMaterialTextures(directory, mat, TextureType.Diffuse, "texture_diffuse");
                    textures.AddRange(diffuseMaps);
                }
                if (mat.HasTextureSpecular)
                {
                    var specularMaps = LoadMaterialTextures(directory, mat, TextureType.Specular, "texture_specular");
                    textures.AddRange(specularMaps);
                }

                // get material color props
                material = new MeshMaterial
                {
                    Ambient = mat.ColorAmbient.ToOpenTK(),
                    Diffuse = mat.ColorDiffuse.ToOpenTK(),
                    Specular = mat.ColorSpecular.ToOpenTK(),
                    Shininess = mat.Shininess
                };
            }

            return new Mesh(vertices.ToArray(), indices.ToArray(), textures.ToArray(), material, mesh.Name);
        }

        private static IEnumerable<MeshTexture> LoadMaterialTextures(string directory, Material mat, TextureType type, string typeName)
        {
            int count = mat.GetMaterialTextureCount(type);
            for (int i = 0; i < count; i++)
            {
                mat.GetMaterialTexture(type, i, out TextureSlot slot);

                var path = $"{directory}\\{slot.FilePath}";
                if (!_texturesLoaded.TryGetValue(path, out var texture))
                    _texturesLoaded[path] = texture = TextureFromFile(path, typeName);

                yield return texture;
            }
        }

        private static MeshTexture TextureFromFile(string path, string typeName)  // [obsolete] TODO: return by ref
        {
            using var io = new FileStream(path, FileMode.Open);

            // load texture file with StbImage
            var img = ImageResult.FromStream(io);

            var texture = new MeshTexture
            {
                Type = typeName,
                Path = path
            };

            var format = img.SourceComp switch
            {
                ColorComponents.Grey => PixelInternalFormat.R8,
                ColorComponents.GreyAlpha => PixelInternalFormat.Rg8,
                ColorComponents.RedGreenBlue => PixelInternalFormat.Rgb8,
                ColorComponents.RedGreenBlueAlpha => PixelInternalFormat.Rgba8,
                _ => throw new ArgumentException("Unknown pixel format.")
            };

            // send necessary actions to dispatcher
            Dispatcher.RenderActions.Enqueue(() =>
            {
                // load and generate the texture
                GL.GenTextures(1, out texture.ID);

                // set texture data
                GL.BindTexture(TextureTarget.Texture2D, texture.ID);
                GL.TexImage2D(TextureTarget.Texture2D, 0, format, img.Width, img.Height, 0, (PixelFormat)format, PixelType.UnsignedByte, img.Data);
                GL.GenerateMipmap(GenerateMipmapTarget.Texture2D);

                // set texture wrapping/filtering options
                GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)OpenTK.Graphics.OpenGL4.TextureWrapMode.Repeat);
                GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)OpenTK.Graphics.OpenGL4.TextureWrapMode.Repeat);
                GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.LinearMipmapLinear);
                GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
            });

            return texture;
        }

        public void Update(Matrix4 state)
        {
            _state = state;
        }

        public void Render(ShaderProgram shader, PrimitiveType type = PrimitiveType.Triangles, int? count = null)
        {
            /* The shader must be enabled by UseProgram() for uniforms to be set.
             * But since every object on the scene is rendered via Model.Render(),
             * it is easier to enable the shader only once here than every time a uniform is being set.
             */
            shader.Use();

            // setup model matrix
            shader.SetMatrix4("model", _state);

            foreach (var mesh in Meshes)
                mesh.Render(shader, type, count, RenderFlags);
        }

        public Model DeepCopy() => new(Meshes.Select(mesh => mesh.DeepCopy()), _state, RenderFlags);

        public void Dispose()  // [obsolete] TODO: fix finalization, it seems to be not proper
        {
            // dispose of all meshes
            foreach (var mesh in Meshes)
                mesh.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
