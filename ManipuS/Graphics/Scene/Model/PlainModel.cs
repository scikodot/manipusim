using System;
using Assimp;
using BulletSharp;
using BulletSharp.Math;
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

        public Material Material { get; private set; }
        //public Collider Collider { get; private set; }

        private Matrix4 _state;
        public ref Matrix4 State => ref _state;

        public PlainModel(float[] vertices, uint[] indices = null, Material material = null, RigidBody rigidBody = null, Matrix4 state = default)
        {
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
                GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(uint), indices, BufferUsageHint.StaticDraw);
            }

            // set all attributes at appropriate locations
            if (material == null)  // TODO: refactor!
            {
                // vertex
                GL.EnableVertexAttribArray(0);
                GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 7 * sizeof(float), 0);

                // color
                GL.EnableVertexAttribArray(2);
                GL.VertexAttribPointer(2, 4, VertexAttribPointerType.Float, false, 7 * sizeof(float), 3 * sizeof(float));
            }
            else
            {
                // vertex
                GL.EnableVertexAttribArray(0);
                GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);

                // normal
                GL.EnableVertexAttribArray(1);
                GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            }

            GL.BindVertexArray(0);

            // set material if presented
            Material = material;

            //// set rigid body is presented
            //RigidBody = rigidBody;

            // set initial state if presented
            State = state == default ? Matrix4.Identity : state;
        }

        public void UpdateData(float[] data)
        {
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);
        }

        //public void UpdateState()
        //{
        //    if (RigidBody != null)
        //        _state = RigidBody.WorldTransform;
        //}

        public void Render(Shader shader, Action render)
        {
            var mat = new Matrix4(
                State.M11, State.M12, State.M13, State.M14,
                State.M21, State.M22, State.M23, State.M24,
                State.M31, State.M32, State.M33, State.M34,
                State.M41, State.M42, State.M43, State.M44);

            // setup model matrix
            shader.SetMatrix4("model", ref mat, true);  // TODO: remove transpose?

            // setup rendering mode
            if (Material == null)
                shader.SetBool("useMaterial", 0);
            else
            {
                shader.SetBool("useMaterial", 1);
                shader.SetBool("useTextures", 0);

                // set colors
                shader.SetVector3("material.ambientCol", new OpenTK.Vector3(Material.ColorAmbient.R, Material.ColorAmbient.G, Material.ColorAmbient.B));
                shader.SetVector3("material.diffuseCol", new OpenTK.Vector3(Material.ColorDiffuse.R, Material.ColorDiffuse.G, Material.ColorDiffuse.B));
                shader.SetVector3("material.specularCol", new OpenTK.Vector3(Material.ColorSpecular.R, Material.ColorSpecular.G, Material.ColorSpecular.B));
                shader.SetFloat("material.shininess", Material.Shininess);
            }

            // render
            GL.BindVertexArray(VAO);
            render();
            GL.BindVertexArray(0);
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
