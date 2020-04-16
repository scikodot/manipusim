using System;
using OpenTK;
using OpenTK.Graphics.OpenGL4;

namespace Graphics
{
    public class Entity  // TODO: add methods for updating entity state, because redefinition is not a good idea
    {
        public int VAO, VBO, EBO;
        public Shader Shader;

        public Entity(Shader shader, float[] data, uint[] indices = null)
        {
            Shader = shader;

            // generating array/buffer objects
            VAO = GL.GenVertexArray();
            VBO = GL.GenBuffer();

            GL.BindVertexArray(VAO);

            // binding vertex data to buffer
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);

            // binding indices data to buffer, if presented
            if (indices != null)
            {
                EBO = GL.GenBuffer();
                GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
                GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(float), indices, BufferUsageHint.StaticDraw);
            }
            else
                EBO = 0;

            // configuring all the needed attributes
            var PosAttrib = Shader.GetAttribLocation("aPos");
            GL.VertexAttribPointer(PosAttrib, 3, VertexAttribPointerType.Float, false, 7 * sizeof(float), 0);
            GL.EnableVertexAttribArray(PosAttrib);

            var ColAttrib = Shader.GetAttribLocation("aColor");
            GL.VertexAttribPointer(ColAttrib, 4, VertexAttribPointerType.Float, false, 7 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(ColAttrib);

            GL.BindVertexArray(0);
        }

        public void Update(float[] data)
        {
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);
        }

        public void Display(Matrix4 model, Action draw, bool transpose = false)
        {
            // displaying entity with the appropriate draw method
            GL.BindVertexArray(VAO);
            Shader.SetMatrix4("model", model, transpose);
            draw();
        }

        //~Entity()
        //{
        //    Dispatcher.ActionsQueue.Enqueue(() =>
        //    {
        //        GL.DeleteBuffer(EBO);
        //        GL.DeleteBuffer(VBO);
        //        GL.DeleteVertexArray(VAO);
        //    });
        //}

        public void Dispose()
        {
            GL.DeleteBuffer(EBO);
            GL.DeleteBuffer(VBO);
            GL.DeleteVertexArray(VAO);
        }

        //public static bool operator ==(Entity e1, Entity e2)
        //{
        //    return e1.VAO == e2.VAO;
        //}

        //public static bool operator !=(Entity e1, Entity e2)
        //{
        //    return e1.VAO != e2.VAO;
        //}
    }
}
