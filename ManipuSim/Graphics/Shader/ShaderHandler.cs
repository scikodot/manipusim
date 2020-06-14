using OpenToolkit.Mathematics;
using OpenToolkit.Graphics.OpenGL4;

namespace Graphics
{
    public static class ShaderHandler
    {
        //private static string VertexShader => InputHandler.ExeDirectory + @"\Graphics\Shader\Shaders\VertexShader.glsl";
        //private static string ComplexFragmentShader => InputHandler.ExeDirectory + @"\Graphics\Shader\Shaders\FragmentShader.glsl";
        //private static string GenericFragmentShader => InputHandler.ExeDirectory + @"\Graphics\Shader\Shaders\LineShader.glsl";

        private static string VertexShader => InputHandler.ExeDirectory + "/Resources/Shaders/VertexShader.glsl";
        private static string ComplexFragmentShader => InputHandler.ExeDirectory + "/Resources/Shaders/FragmentShader.glsl";
        private static string GenericFragmentShader => InputHandler.ExeDirectory + "/Resources/Shaders/LineShader.glsl";

        public static Shader GenericShader { get; private set; }
        public static Shader ComplexShader { get; private set; }

        public static void InitializeShaders()
        {
            // create shaders
            GenericShader = new Shader(VertexShader, GenericFragmentShader);
            ComplexShader = new Shader(VertexShader, ComplexFragmentShader);
        }

        public static void SetupShaders(Camera camera)
        {
            // setup generic shader
            GenericShader.Use();
            GenericShader.SetMatrix4("view", ref camera.ViewMatrix, false);
            GenericShader.SetMatrix4("projection", ref camera.ProjectionMatrix, false);
            GenericShader.SetVector3("color", Vector3.One);

            // setup complex shader
            ComplexShader.Use();

            // set view and projection matrices;
            // these matrices come pre-transposed, so there's no need to transpose them again (see VertexShader file)
            ComplexShader.SetMatrix4("view", ref camera.ViewMatrix, false);
            ComplexShader.SetMatrix4("projection", ref camera.ProjectionMatrix, false);

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

        public static void Dispose()
        {
            // delete shader programs
            GL.DeleteProgram(ComplexShader.Handle);
            GL.DeleteProgram(GenericShader.Handle);
        }
    }
}
