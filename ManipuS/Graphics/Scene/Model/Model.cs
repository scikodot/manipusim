using System;
using System.Collections.Generic;

using OpenTK;
using OpenTK.Graphics.OpenGL4;
using Assimp;
using StbImageSharp;

namespace Graphics
{
    public class Model : IDisposable
    {
        private static readonly List<MeshTexture> TexturesLoaded = new List<MeshTexture>();
        public List<Mesh> Meshes { get; } = new List<Mesh>();

        public string Directory { get; private set; }  // TODO: remove

        private Matrix4 _state;
        public ref Matrix4 State => ref _state;

        // create a model, consisting of a single mesh
        //public ComplexModel(System.Numerics.Vector3[] vertices, uint[] indices = null, MeshMaterial material = default, Matrix4 state = default, string name = "")
        //{
        //    Meshes.Add(new Mesh(name, MeshVertex.Convert(vertices), indices ?? new uint[0], new MeshTexture[0], material));  // TODO: perhaps replace empty array with nulls?

        //    State = state == default ? Matrix4.Identity : state;
        //}

        public Model(MeshVertex[] vertices, uint[] indices = null, MeshMaterial material = default, Matrix4 state = default, string name = "")
        {
            Meshes.Add(new Mesh(name, vertices, indices ?? new uint[0], new MeshTexture[0], material));  // TODO: perhaps replace empty array with nulls?

            State = state == default ? Matrix4.Identity : state;
        }

        // load an arbitrary model, located at the specified path
        public Model(string path)
        {
            LoadModel(path);
        }

        //public void Update(int meshIndex, float[] data, uint[] indices)
        //{
        //    Meshes[meshIndex].Update(data, indices);
        //}

        //public void Update(int meshIndex, System.Numerics.Vector3[] data, uint[] indices)
        //{
        //    Meshes[meshIndex].Update(MeshVertex.Convert(data), indices);
        //}

        public void Update(int meshIndex, MeshVertex[] data, uint[] indices)
        {
            Meshes[meshIndex].Update(data, indices);
        }

        public void Render(Shader shader, MeshMode mode, Action render = default)
        {
            // setup model matrix
            shader.SetMatrix4("model", ref State, true);

            foreach (var mesh in Meshes)
                mesh.Render(shader, mode, render);
        }

        private void LoadModel(string path)
        {
            var importer = new AssimpContext();
            var scene = importer.ImportFile(path, PostProcessSteps.Triangulate | PostProcessSteps.FlipUVs);

            if (scene == null || scene.RootNode == null || (scene.SceneFlags & SceneFlags.Incomplete) == SceneFlags.Incomplete)
            {
                // TODO: throw exception in case of an unsuccessful scene import
                return;
            }

            Directory = path.Substring(0, path.LastIndexOfAny(new char[] { '/', '\\' }));

            ProcessNode(scene.RootNode, scene);
        }

        private void ProcessNode(Node node, Scene scene)
        {
            // process all the node's meshes (if any)
            for (int i = 0; i < node.MeshCount; i++)
            {
                Assimp.Mesh mesh = scene.Meshes[node.MeshIndices[i]];
                Meshes.Add(ProcessMesh(mesh, scene));
            }

            // then do the same for each of its children
            for (int i = 0; i < node.ChildCount; i++)
            {
                ProcessNode(node.Children[i], scene);
            }
        }

        private Mesh ProcessMesh(Assimp.Mesh mesh, Scene scene)
        {
            var vertices = new List<MeshVertex>();
            var indices = new List<uint>();
            var textures = new List<MeshTexture>();
            var color = new MeshMaterial();

            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var vertex = new MeshVertex();

                // process vertex positions, normals and texture coordinates
                var pos = mesh.Vertices[i];
                vertex.Position = new Vector3(pos.X, pos.Y, pos.Z);

                var norm = mesh.Normals[i];
                vertex.Normal = new Vector3(norm.X, norm.Y, norm.Z);

                if (mesh.HasTextureCoords(0))  // does the mesh contain texture coordinates?
                {
                    var tex = mesh.TextureCoordinateChannels[0][i];
                    vertex.TexCoords = new Vector2(tex.X, tex.Y);
                }

                vertices.Add(vertex);
            }

            // process indices
            for (int i = 0; i < mesh.FaceCount; i++)
            {
                Face face = mesh.Faces[i];
                for (int j = 0; j < face.IndexCount; j++)
                    indices.Add((uint)face.Indices[j]);
            }

            // process material
            if (mesh.MaterialIndex >= 0)
            {
                Material material = scene.Materials[mesh.MaterialIndex];

                // get all the needed material textures
                if (material.HasTextureDiffuse)
                {
                    List<MeshTexture> diffuseMaps = LoadMaterialTextures(material, TextureType.Diffuse, "texture_diffuse");
                    textures.AddRange(diffuseMaps);
                }
                if (material.HasTextureSpecular)
                {
                    List<MeshTexture> specularMaps = LoadMaterialTextures(material, TextureType.Specular, "texture_specular");
                    textures.AddRange(specularMaps);
                }

                // get all the needed material colors (default values if they're not presented)
                color.Ambient = new Vector4(
                    material.ColorAmbient.R, 
                    material.ColorAmbient.G, 
                    material.ColorAmbient.B, 
                    material.ColorAmbient.A);
                color.Diffuse = new Vector4(
                    material.ColorDiffuse.R, 
                    material.ColorDiffuse.G, 
                    material.ColorDiffuse.B, 
                    material.ColorDiffuse.A);
                color.Specular = new Vector4(
                    material.ColorSpecular.R, 
                    material.ColorSpecular.G, 
                    material.ColorSpecular.B, 
                    material.ColorSpecular.A);
                color.Shininess = material.Shininess;
            }

            return new Mesh(mesh.Name, vertices.ToArray(), indices.ToArray(), textures.ToArray(), color);
        }

        private List<MeshTexture> LoadMaterialTextures(Material mat, TextureType type, string typeName)
        {
            var textures = new List<MeshTexture>();
            for (int i = 0; i < mat.GetMaterialTextureCount(type); i++)
            {
                mat.GetMaterialTexture(type, i, out TextureSlot slot);

                var texLoaded = TexturesLoaded.Find(t => t.Path == slot.FilePath);
                if (texLoaded == null)
                {
                    MeshTexture texture = TextureFromFile(slot.FilePath, Directory, typeName);
                    textures.Add(texture);
                    TexturesLoaded.Add(texture);  // add to loaded textures
                }
                else
                    textures.Add(texLoaded);
            }

            return textures;
        }

        private MeshTexture TextureFromFile(string filename, string directory, string typeName)  // TODO: return by ref
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

        public Model ShallowCopy()
        {
            return (Model)MemberwiseClone();
        }

        public void Dispose()  // TODO: fix finalization, it seems to be not proper
        {
            // dispose all meshes
            foreach (var mesh in Meshes)
            {
                mesh.Dispose();
            }
        }
    }
}
