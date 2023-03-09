using System;
using System.IO;
using System.Text;
using System.Collections.Generic;

using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;
using System.Linq;

namespace Graphics
{
    public class ShaderProgram : IDisposable
    {
        private readonly int _handle;
        private readonly Dictionary<string, int> _uniformLocations = new();

        public ShaderProgram(params (ShaderType type, string path)[] shaders)
        {
            _handle = GL.CreateProgram();

            // create shaders from source
            var ids = shaders.Select(s => CreateShader(s.type, s.path));

            // attach them to the program
            foreach (var id in ids)
                GL.AttachShader(_handle, id);

            // link the program
            Link();

            // shaders are not needed anymore, so detach and delete
            foreach (var id in ids)
            {
                GL.DetachShader(_handle, id);
                GL.DeleteShader(id);
            }

            // cache the uniforms
            GL.GetProgram(_handle, GetProgramParameterName.ActiveUniforms, out var uniformCount);
            for (var i = 0; i < uniformCount; i++)
            {
                var uniform = GL.GetActiveUniform(_handle, i, out _, out _);
                var location = GL.GetUniformLocation(_handle, uniform);
                _uniformLocations.Add(uniform, location);
            }
        }

        private static int CreateShader(ShaderType type, string path)
        {
            var shader = GL.CreateShader(type);
            LoadShader(shader, path);
            CompileShader(shader);
            return shader;
        }

        private static void LoadShader(int shader, string path)
        {
            using var sr = new StreamReader(path, Encoding.UTF8);
            GL.ShaderSource(shader, sr.ReadToEnd());
        }

        private static void CompileShader(int shader)
        {
            GL.CompileShader(shader);

            // check for compilation errors
            GL.GetShader(shader, ShaderParameter.CompileStatus, out var code);
            if (code != (int)All.True)
            {
                string info = string.Format("Info: {0}", GL.GetShaderInfoLog(shader));
                throw new Exception($"Error occurred whilst compiling Shader({shader}):\n{info}");
            }
        }

        private void Link()
        {
            GL.LinkProgram(_handle);

            // check for linking errors
            GL.GetProgram(_handle, GetProgramParameterName.LinkStatus, out var code);
            if (code != (int)All.True)
            {
                string info = string.Format("Info: {0}", GL.GetProgramInfoLog(_handle));
                throw new Exception($"Error occurred whilst linking Program({_handle}):\n{info}");
            }
        }

        public void Use()
        {
            GL.UseProgram(_handle);
        }

        // The shader sources provided with this project use hardcoded layout(location)-s. If you want to do it dynamically,
        // you can omit the layout(location=X) lines in the vertex shader, and use this in VertexAttribVector3er instead of the hardcoded values.
        public int GetAttribLocation(string attribName)
        {
            return GL.GetAttribLocation(_handle, attribName);
        }

        private void EnsureUniformExists(string name)
        {
            if (!_uniformLocations.ContainsKey(name))
                throw new ArgumentException($"No such uniform: {name}.");
        }

        public void SetInt(string name, int data)
        {
            EnsureUniformExists(name);
            GL.Uniform1(_uniformLocations[name], data);
        }

        public void SetFloat(string name, float data)
        {
            EnsureUniformExists(name);
            GL.Uniform1(_uniformLocations[name], data);
        }

        public void SetBool(string name, uint data)
        {
            EnsureUniformExists(name);
            GL.Uniform1(_uniformLocations[name], data);
        }
        public void SetMatrix4(string name, Matrix4 data)
        {
            EnsureUniformExists(name);
            GL.UniformMatrix4(_uniformLocations[name], false, ref data);
        }

        public void SetVector3(string name, Vector3 data)
        {
            EnsureUniformExists(name);
            GL.Uniform3(_uniformLocations[name], data);
        }

        public void SetVector4(string name, Vector4 data)
        {
            EnsureUniformExists(name);
            GL.Uniform4(_uniformLocations[name], data);
        }

        public void SetColor4(string name, Color4 data)
        {
            EnsureUniformExists(name);
            GL.Uniform4(_uniformLocations[name], data);
        }

        public void Dispose()
        {
            GL.DeleteProgram(_handle);

            GC.SuppressFinalize(this);
        }
    }
}