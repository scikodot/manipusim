using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;
using System;

namespace Graphics
{
    public class ShaderHandler : IDisposable
    {
        private readonly MainWindow _parent;

        public ShaderProgram MainShader { get; private set; }

        public ShaderHandler(MainWindow parent)
        {
            _parent = parent;

            // create shaders

            MainShader = new ShaderProgram(
                (ShaderType.VertexShader, _parent.InputHandler.VertexShader),
                (ShaderType.FragmentShader, _parent.InputHandler.ComplexFragmentShader));
        }

        public void SetupShaders(Camera camera)
        {
            MainShader.Use();

            // set view and projection matrices;
            // these matrices come pre-transposed, so there's no need to transpose them again (see VertexShader file)
            MainShader.SetMatrix4("view", camera.ViewMatrix);
            MainShader.SetMatrix4("projection", camera.ProjectionMatrix);

            // set general properties
            MainShader.SetVector3("viewPos", camera.Position);

            // set directional light properties
            MainShader.SetVector3("dirLight[0].direction", new Vector3(1.0f, -0.2f, -1.0f));
            MainShader.SetVector3("dirLight[1].direction", new Vector3(-1.0f, -0.5f, 0.0f));
            MainShader.SetVector3("dirLight[2].direction", new Vector3(0.3f, -0.8f, 0.7f));
            for (int i = 0; i < 3; i++)
            {
                MainShader.SetVector3($"dirLight[{i}].ambient", new Vector3(0.05f, 0.05f, 0.05f));
                MainShader.SetVector3($"dirLight[{i}].diffuse", new Vector3(0.7f, 0.7f, 0.7f));
                MainShader.SetVector3($"dirLight[{i}].specular", new Vector3(0.2f, 0.2f, 0.2f));
            }
        }

        public void Dispose()
        {
            MainShader.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
