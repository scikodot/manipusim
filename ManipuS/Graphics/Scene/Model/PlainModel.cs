using System;

using OpenTK;
using OpenTK.Graphics.OpenGL4;

namespace Graphics
{
    //abstract class AnyModel
    //{
    //    public abstract Matrix4 State { get; set; }

    //    public abstract void Draw(Shader shader);

    //    public static AnyModel Create(string path)
    //    {
    //        return new ComplexModel(path);
    //    }

    //    public static AnyModel Create(Shader shader, float[] data, uint[] indices)
    //    {
    //        return new PlainModel(shader, data, indices, null);
    //    }

    //    public static AnyModel Create(Shader shader, float[] data, Action draw)
    //    {
    //        return new PlainModel(shader, data, null, draw);
    //    }
    //}

    public class PlainModel : IRenderable, IDisposable
    {
        private readonly int VAO, VBO, EBO;

        private Matrix4 _state;
        public ref Matrix4 State => ref _state;

        public PlainModel(float[] vertices, uint[] indices = null, Matrix4 state = default)
        {
            State = state == default ? Matrix4.Identity : state;

            // generate array/buffer objects
            VAO = GL.GenVertexArray();
            VBO = GL.GenBuffer();

            GL.BindVertexArray(VAO);

            // bind vertices to buffer
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, vertices.Length * sizeof(float), vertices, BufferUsageHint.StaticDraw);

            // bind indices to buffer, if presented
            if (indices != null)
            {
                EBO = GL.GenBuffer();
                GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
                GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(float), indices, BufferUsageHint.StaticDraw);
            }

            // set all attributes at appropriate locations
            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 7 * sizeof(float), 0);

            GL.EnableVertexAttribArray(3);
            GL.VertexAttribPointer(3, 4, VertexAttribPointerType.Float, false, 7 * sizeof(float), 3 * sizeof(float));

            GL.BindVertexArray(0);
        }

        public void Update(float[] data)
        {
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);
        }

        public void Render(Shader shader, Action render)
        {
            // render model with the appropriate draw method
            GL.BindVertexArray(VAO);
            shader.SetMatrix4("model", ref State, true);  // TODO: remove transpose?
            render();
        }

        public void Dispose()
        {
            // dispose the model
            Dispose(true);

            // suppress additional finalization
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            // TODO: check for disposed; see documentation

            // clear unmanaged resources
            GL.DeleteBuffer(EBO);
            GL.DeleteBuffer(VBO);
            GL.DeleteVertexArray(VAO);

            Console.WriteLine($"Disposed model: VAO - {VAO}, VBO - {VBO}");
        }

        // TODO: is there any need in finalizer?
        //~PlainModel()
        //{
        //    // dispose the model without clearing managed resources, that is performed by GC
        //    Dispose(false);
        //}
    }
}
