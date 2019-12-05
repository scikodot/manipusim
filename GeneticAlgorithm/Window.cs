using System;
using System.Collections.Generic;
using System.Threading;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Input;
using Helper;
using GeneticAlgorithm;

namespace RoboDraw
{
    public class Window : GameWindow
    {
        private float[] linesX =
        {
            10.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
            -10.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        };

        private float[] linesY =
        {
            0.0f, 10.0f, 0.0f, 1.0f, 1.0f, 1.0f,
            0.0f, -10.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        };
        
        private int vbo_frameX, vbo_frameY, vbo_joints, vbo_path, vbo_goal;
        private int vao_frameX, vao_frameY, vao_joints, vao_path, vao_goal;
        private int[] vbo_obst, vbo_bound, vbo_chs;
        private int[] vao_obst, vao_bound, vao_chs;
        private List<int> vbo_tree, vao_tree;

        private Shader _shader;
        
        private Camera _camera;
        private bool _firstMove = true;
        private Vector2 _lastPos;

        private double _time;
        
        private int Count = 0, MainCount = 0;
        
        private Thread Main;

        public Window(int width, int height, string title) : base(width, height, GraphicsMode.Default, title) { }


        protected override void OnLoad(EventArgs e)
        {
            GL.ClearColor(0.2f, 0.3f, 0.3f, 1.0f);

            GL.Enable(EnableCap.DepthTest);

            _shader = new Shader(@"C:\Users\Dan\source\repos\GeneticAlgorithm\Shaders\VertexShader.txt",
                @"C:\Users\Dan\source\repos\GeneticAlgorithm\Shaders\FragmentShader.txt");
            _shader.Use();

            // We initialize the camera so that it is 3 units back from where the rectangle is
            // and give it the proper aspect ratio
            _camera = new Camera(Vector3.UnitZ * 6, Width / (float)Height);

            // We make the mouse cursor invisible so we can have proper FPS-camera movement
            CursorVisible = false;

            // retrieving genetic algorithm resultant trajectory
            Main = new Thread(Manager.Execute)
            {
                Name = "Manager",
                IsBackground = true
            };
            Main.Start();

            // coordinate frame lines
            SetData(ref vbo_frameX, ref vao_frameX, linesX);
            SetData(ref vbo_frameY, ref vao_frameY, linesY);

            base.OnLoad(e);
        }


        protected override void OnRenderFrame(FrameEventArgs e)
        {
            _time += 4.0 * e.Time;

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            _shader.Use();
            _shader.SetBool("use_color", 1);

            _shader.SetMatrix4("view", _camera.GetViewMatrix());
            _shader.SetMatrix4("projection", _camera.GetProjectionMatrix());

            Matrix4 model;

            // lines on the X axis
            GL.BindVertexArray(vao_frameX);
            model = Matrix4.Identity;
            _shader.SetMatrix4("model", model);
            GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
            //for (int i = 1; i < 5; i++)
            //{
            //    model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitZ * i);
            //    _shader.SetMatrix4("model", model);
            //    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);

            //    model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitZ * -i);
            //    _shader.SetMatrix4("model", model);
            //    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
            //}

            // lines on the Y axis
            GL.BindVertexArray(vao_frameY);
            model = Matrix4.Identity;
            _shader.SetMatrix4("model", model);
            GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
            //for (int i = 1; i < 5; i++)
            //{
            //    model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitX * i);
            //    _shader.SetMatrix4("model", model);
            //    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);

            //    model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitX * -i);
            //    _shader.SetMatrix4("model", model);
            //    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
            //}

            // goal
            if (vao_goal == 0)
            {
                if (Manager.States["Goal"])
                    SetData(ref vbo_goal, ref vao_goal, GL_Convert(new Point[] { Manager.Goal }, new Vector3(1, 1, 0)));
            }
            else
            {
                model = Matrix4.Identity;
                DisplayData(vao_goal, model, () =>
                {
                    GL.PointSize(5);
                    GL.DrawArrays(PrimitiveType.Points, 0, 1);
                    GL.PointSize(1);
                });
            }

            // obstacles + boundings
            if (vbo_obst == null)
            {
                if (Manager.States["Obstacles"])
                {
                    vbo_obst = new int[Manager.Obstacles.Length];
                    vao_obst = new int[Manager.Obstacles.Length];
                    vbo_bound = new int[Manager.Obstacles.Length];
                    vao_bound = new int[Manager.Obstacles.Length];
                    for (int i = 0; i < vbo_obst.Length; i++)
                    {
                        SetData(ref vbo_obst[i], ref vao_obst[i], GL_Convert(Manager.Obstacles[i].Data, Vector3.One));
                        SetData(ref vbo_bound[i], ref vao_bound[i], GL_Convert(Manager.Obstacles[i].Bounding, Vector3.UnitY));
                    }
                }
            }
            else
            {
                model = Matrix4.Identity;
                for (int i = 0; i < vao_obst.Length; i++)
                {
                    DisplayData(vao_obst[i], model, () =>
                    {
                        GL.DrawArrays(PrimitiveType.Points, 0, Manager.Obstacles[i].Data.Length);
                    });
                }
                
                for (int i = 0; i < vao_bound.Length; i++)
                {
                    DisplayData(vao_bound[i], model, () =>
                    {
                        GL.DrawArrays(PrimitiveType.LineLoop, 0, Manager.Obstacles[i].Bounding.Length);
                    });
                }
            }

            // chromosomes
            if (PathPlanner.Chs != null)
            {
                vbo_chs = new int[PathPlanner.Chs.Length];
                vao_chs = new int[PathPlanner.Chs.Length];
                for (int i = 0; i < vbo_chs.Length; i++)
                {
                    SetData(ref vbo_chs[i], ref vao_chs[i], GL_Convert(PathPlanner.Chs[i].Genes, new Vector3(1 - i / (2 * PathPlanner.Chs.Length), 0.0f, 0.0f)));

                    model = Matrix4.Identity;
                    DisplayData(vao_chs[i], model, () =>
                    {
                        GL.DrawArrays(PrimitiveType.LineStrip, 0, PathPlanner.ParamNum);
                    });
                }
            }

            // checking if the thread has aborted
            if (!Main.IsAlive)
            {
                // path
                if (vao_path == 0)
                {
                    if (Manager.States["Path"])
                        SetData(ref vbo_path, ref vao_path, GL_Convert(Manager.Path.ToArray(), Vector3.UnitX));
                }
                else
                {
                    model = Matrix4.Identity;
                    DisplayData(vao_path, model, () =>
                    {
                        GL.DrawArrays(PrimitiveType.LineStrip, 0, Manager.Path.Count);
                    });
                }

                // manipulator
                if (Manager.States["Joints"])
                {
                    /*if (++MainCount % 30 == 0)
                    {
                        SetData(ref vbo_joints, ref vao_joints, GL_Convert(Manager.Joints[Count < Manager.Joints.Count - 1 ? Count++ : Count], new Vector3(1.0f, 0.5f, 0.0f)));
                    }
                    else
                    {
                        SetData(ref vbo_joints, ref vao_joints, GL_Convert(Manager.Joints[Count], new Vector3(1.0f, 0.5f, 0.0f)));
                    }*/

                    if (Manager.Joints.Count != 0)
                        SetData(ref vbo_joints, ref vao_joints, GL_Convert(Manager.Joints[Count < Manager.Joints.Count - 1 ? Count++ : Count], new Vector3(1.0f, 0.5f, 0.0f)));

                    model = Matrix4.Identity;
                    DisplayData(vao_joints, model, () =>
                    {
                        GL.DrawArrays(PrimitiveType.LineStrip, 0, Manager.Manip.Links.Length + 1);
                    });
                }

                // random Tree
                if (vao_tree == null)
                {
                    if (Manager.States["Tree"])
                    {
                        vbo_tree = new List<int>();
                        vao_tree = new List<int>();
                        for (int i = 1; i < Manager.Tree.Layers.Count; i++)
                        {
                            for (int j = 0; j < Manager.Tree.Layers[i].Count; j++)
                            {
                                int b = 0, a = 0;
                                SetData(ref b, ref a, GL_Convert(new Point[] { Manager.Tree.Layers[i][j].p, Manager.Tree.Layers[i][j].Parent.p }, Vector3.Zero));
                                vbo_tree.Add(b);
                                vao_tree.Add(a);
                            }
                        }
                    }
                }
                else
                {
                    model = Matrix4.Identity;
                    for (int i = 0; i < Manager.Tree.Count - 1; i++)
                    {
                        DisplayData(vao_tree[i], model, () =>
                        {
                            GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                        });
                    }
                }
            }

            SwapBuffers();

            base.OnRenderFrame(e);
        }


        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            if (!Focused) // check to see if the window is focused
            {
                return;
            }

            var input = Keyboard.GetState();

            if (input.IsKeyDown(Key.Escape))
            {
                Exit();
            }

            const float cameraSpeed = 3f;
            const float sensitivity = 0.2f;

            if (input.IsKeyDown(Key.W))
                _camera.Position += _camera.Up * cameraSpeed * (float)e.Time; // Up 
            if (input.IsKeyDown(Key.S))
                _camera.Position -= _camera.Up * cameraSpeed * (float)e.Time; // Down
            if (input.IsKeyDown(Key.A))
                _camera.Position -= _camera.Right * cameraSpeed * (float)e.Time; // Left
            if (input.IsKeyDown(Key.D))
                _camera.Position += _camera.Right * cameraSpeed * (float)e.Time; // Right

            // Get the mouse state
            var mouse = Mouse.GetState();

            if (_firstMove) // this bool variable is initially set to true
            {
                _lastPos = new Vector2(mouse.X, mouse.Y);
                _firstMove = false;
            }
            else
            {
                // Calculate the offset of the mouse position
                var deltaX = mouse.X - _lastPos.X;
                var deltaY = mouse.Y - _lastPos.Y;
                _lastPos = new Vector2(mouse.X, mouse.Y);

                // Apply the camera pitch and yaw (we clamp the pitch in the camera class)
                //_camera.Yaw += deltaX * sensitivity;
                //_camera.Pitch -= deltaY * sensitivity; // reversed since y-coordinates range from bottom to top
            }

            base.OnUpdateFrame(e);
        }
        
        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            if (Focused) // check to see if the window is focused
            {
                Mouse.SetPosition(X + Width / 2f, Y + Height / 2f);  // set mouse position back to center
            }

            base.OnMouseMove(e);
        }
        
        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            _camera.Fov -= e.DeltaPrecise;  // zooming
            base.OnMouseWheel(e);
        }


        protected override void OnResize(EventArgs e)
        {
            GL.Viewport(0, 0, Width, Height);
            // We need to update the aspect ratio once the window has been resized
            _camera.AspectRatio = Width / (float)Height;
            base.OnResize(e);
        }


        protected override void OnUnload(EventArgs e)
        {
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindVertexArray(0);
            GL.UseProgram(0);

            GL.DeleteProgram(_shader.Handle);

            base.OnUnload(e);
        }

        protected void SetData(ref int VBO, ref int VAO, float[] data)
        {
            //generating array/buffer objects in case they haven't been generated yet
            if (VAO == 0)
                VAO = GL.GenVertexArray();
            if (VBO == 0)
                VBO = GL.GenBuffer();
            
            GL.BindVertexArray(VAO);

            //binding data to buffer
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
            GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);

            //configuring all the needed attributes
            var attrib1 = _shader.GetAttribLocation("aPos");
            GL.VertexAttribPointer(attrib1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(attrib1);

            var attrib2 = _shader.GetAttribLocation("aColor");
            GL.VertexAttribPointer(attrib2, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(attrib2);

            GL.BindVertexArray(0);
        }

        protected void DisplayData(int VAO, Matrix4 model, Action draw)
        {
            GL.BindVertexArray(VAO);
            _shader.SetMatrix4("model", model);
            draw();
        }

        protected float[] GL_Convert(Point[] data, Vector3 color)
        {
            float[] res = new float[data.Length * 6];

            for (int i = 0; i < data.Length; i++)
            {
                res[6 * i] = (float)data[i].x;
                res[6 * i + 1] = (float)data[i].y;
                res[6 * i + 2] = 0;
                res[6 * i + 3] = color.X;
                res[6 * i + 4] = color.Y;
                res[6 * i + 5] = color.Z;
            }

            return res;
        }
    }
}