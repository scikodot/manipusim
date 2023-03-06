using System;
using System.Collections.Generic;
using System.Linq;

using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;
using Assimp;
using StbImageSharp;
using System.Data;
using static OpenTK.Graphics.OpenGL.GL;

namespace Graphics
{
    [Flags]
    public enum RenderFlags
    {
        Solid = 1,
        Wireframe = 2,
        Lighting = 4,
        Selected = 8
    }

    public class Model : IDisposable
    {
        private static readonly List<MeshTexture> _texturesLoaded = new();

        private string _directory;

        public List<Mesh> Meshes { get; } = new();

        private Matrix4 _state;
        public Matrix4 State => _state;

        public RenderFlags RenderFlags { get; set; } = RenderFlags.Solid;
        public bool IsSetup => Meshes.All(mesh => mesh.IsSetup);

        public Model(MeshVertex[] vertices, uint[] indices = null, MeshMaterial? material = null, Matrix4? state = null, 
            string name = null, RenderFlags renderFlags = RenderFlags.Solid)
        {
            Meshes.Add(new Mesh(vertices, indices, material: material, name: name));

            _state = state ?? Matrix4.Identity;

            RenderFlags = renderFlags;
        }

        public Model(IEnumerable<Mesh> meshes, Matrix4? state = null, RenderFlags renderFlags = RenderFlags.Solid)
        {
            Meshes.AddRange(meshes);

            _state = state ?? Matrix4.Identity;

            RenderFlags = renderFlags;
        }

        public Model(string path)
        {
            LoadModel(path);
        }

        private void LoadModel(string path)
        {
            using var importer = new AssimpContext();
            var scene = importer.ImportFile(path, PostProcessSteps.Triangulate | PostProcessSteps.FlipUVs);
            if (scene?.RootNode == null || scene.SceneFlags.HasFlag(SceneFlags.Incomplete))
            {
                // TODO: throw exception in case of an unsuccessful scene import
                return;
            }

            _directory = path.Substring(0, path.LastIndexOfAny(new char[] { '/', '\\' }));

            ProcessNode(scene, scene.RootNode);
        }

        private void ProcessNode(Scene scene, Node node)
        {
            // process all node's meshes, if any
            foreach (var index in node.MeshIndices)
                Meshes.Add(ProcessMesh(scene, scene.Meshes[index]));

            // process all descendant nodes
            foreach (var child in node.Children)
                ProcessNode(scene, child);
        }

        private Mesh ProcessMesh(Scene scene, Assimp.Mesh mesh)
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
                    var diffuseMaps = LoadMaterialTextures(mat, TextureType.Diffuse, "texture_diffuse");
                    textures.AddRange(diffuseMaps);
                }
                if (mat.HasTextureSpecular)
                {
                    var specularMaps = LoadMaterialTextures(mat, TextureType.Specular, "texture_specular");
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

        private List<MeshTexture> LoadMaterialTextures(Material mat, TextureType type, string typeName)
        {
            var textures = new List<MeshTexture>();
            for (int i = 0; i < mat.GetMaterialTextureCount(type); i++)
            {
                mat.GetMaterialTexture(type, i, out TextureSlot slot);

                var texLoaded = _texturesLoaded.Find(t => t.Path == slot.FilePath);
                if (texLoaded == null)
                {
                    MeshTexture texture = TextureFromFile(slot.FilePath, _directory, typeName);
                    textures.Add(texture);
                    _texturesLoaded.Add(texture);  // add to loaded textures
                }
                else
                    textures.Add(texLoaded);
            }

            return textures;
        }

        private MeshTexture TextureFromFile(string filename, string directory, string typeName)  // [obsolete] TODO: return by ref
        {
            string resPath = directory + @"\" + filename;

            var texture = new MeshTexture
            {
                Type = typeName,
                Path = filename
            };

            // load texture file with StbImage
            var io = new System.IO.FileStream(resPath, System.IO.FileMode.Open);
            var resLoad = ImageResult.FromStream(io);

            int width = resLoad.Width, height = resLoad.Height;
            ColorComponents nrComponents = resLoad.SourceComp;

            if (resLoad != null)
            {
                PixelInternalFormat format = 0;
                if (nrComponents == ColorComponents.Grey)
                    format = PixelInternalFormat.CompressedRed;
                else if (nrComponents == ColorComponents.RedGreenBlue)
                    format = PixelInternalFormat.Rgb;
                else if (nrComponents == ColorComponents.RedGreenBlueAlpha)
                    format = PixelInternalFormat.Rgba;

                // send necessary actions to dispatcher
                Dispatcher.RenderActions.Enqueue(() =>
                {
                    // load and generate the texture
                    GL.GenTextures(1, out texture.ID);

                    GL.BindTexture(TextureTarget.Texture2D, texture.ID);
                    GL.TexImage2D(TextureTarget.Texture2D, 0, format, width, height, 0, (PixelFormat)format, PixelType.UnsignedByte, resLoad.Data);
                    GL.GenerateMipmap(GenerateMipmapTarget.Texture2D);

                    // set the texture wrapping/filtering options (on the currently bound texture object)
                    GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)OpenTK.Graphics.OpenGL4.TextureWrapMode.Repeat);
                    GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)OpenTK.Graphics.OpenGL4.TextureWrapMode.Repeat);
                    GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.LinearMipmapLinear);
                    GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
                });
            }
            else
            {
                // TODO: throw new exception, although not necessarily - it's already implemented in ImageResult.FromResult
            }

            // explicitly destroy I/O stream object
            io.Dispose();

            return texture;
        }

        public void Update(Matrix4 state)
        {
            _state = state;
        }

        public void Render(Shader shader, Action render = default)
        {
            // setup model matrix
            shader.SetMatrix4("model", ref _state);

            foreach (var mesh in Meshes)
                mesh.Render(shader, RenderFlags, render);
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
