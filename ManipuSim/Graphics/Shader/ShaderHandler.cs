using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;

namespace Graphics
{
    public class ShaderHandler
    {
        private readonly MainWindow _parent;

        public Shader GenericShader { get; private set; }
        public Shader ComplexShader { get; private set; }

        public ShaderHandler(MainWindow parent)
        {
            _parent = parent;

            // create shaders

            GenericShader = new Shader(_parent.InputHandler.VertexShader, 
                                       _parent.InputHandler.GenericFragmentShader);
            ComplexShader = new Shader(_parent.InputHandler.VertexShader, 
                                       _parent.InputHandler.ComplexFragmentShader);
        }

        public void SetupShaders(Camera camera)
        {
            // setup generic shader
            GenericShader.Use();
            GenericShader.SetMatrix4("view", ref camera.ViewMatrix);
            GenericShader.SetMatrix4("projection", ref camera.ProjectionMatrix);
            GenericShader.SetVector3("color", Vector3.One);

            // setup complex shader
            ComplexShader.Use();

            // set view and projection matrices;
            // these matrices come pre-transposed, so there's no need to transpose them again (see VertexShader file)
            ComplexShader.SetMatrix4("view", ref camera.ViewMatrix);
            ComplexShader.SetMatrix4("projection", ref camera.ProjectionMatrix);

            // set general properties
            ComplexShader.SetVector3("viewPos", camera.Position);

            // set directional light properties
            ComplexShader.SetVector3("dirLight[0].direction", new Vector3(1.0f, -0.2f, -1.0f));
            ComplexShader.SetVector3("dirLight[1].direction", new Vector3(-1.0f, -0.5f, 0.0f));
            ComplexShader.SetVector3("dirLight[2].direction", new Vector3(0.3f, -0.8f, 0.7f));
            for (int i = 0; i < 3; i++)
            {
                ComplexShader.SetVector3($"dirLight[{i}].ambient", new Vector3(0.05f, 0.05f, 0.05f));
                ComplexShader.SetVector3($"dirLight[{i}].diffuse", new Vector3(0.7f, 0.7f, 0.7f));
                ComplexShader.SetVector3($"dirLight[{i}].specular", new Vector3(0.2f, 0.2f, 0.2f));
            }
        }

        public void Dispose()
        {
            // delete shader programs
            GL.DeleteProgram(ComplexShader.Handle);
            GL.DeleteProgram(GenericShader.Handle);
        }
    }
}
