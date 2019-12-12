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
        public class Entity
        {
            public int VBO, EBO, VAO;

            public Entity(float[] data, uint[] indices = null)
            {
                //generating array/buffer objects
                VBO = GL.GenBuffer();
                VAO = GL.GenVertexArray();

                GL.BindVertexArray(VAO);

                //binding vertex data to buffer
                GL.BindBuffer(BufferTarget.ArrayBuffer, VBO);
                GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);

                //binding indices data to buffer, if presented
                if (indices != null)
                {
                    EBO = GL.GenBuffer();
                    GL.BindBuffer(BufferTarget.ElementArrayBuffer, EBO);
                    GL.BufferData(BufferTarget.ElementArrayBuffer, indices.Length * sizeof(float), indices, BufferUsageHint.StaticDraw);
                }

                //configuring all the needed attributes
                var PosAttrib = _shader.GetAttribLocation("aPos");
                GL.VertexAttribPointer(PosAttrib, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
                GL.EnableVertexAttribArray(PosAttrib);

                var ColAttrib = _shader.GetAttribLocation("aColor");
                GL.VertexAttribPointer(ColAttrib, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
                GL.EnableVertexAttribArray(ColAttrib);

                GL.BindVertexArray(0);
            }

            public void Display(Matrix4 model, Action draw)
            {
                GL.BindVertexArray(VAO);
                _shader.SetMatrix4("model", model);
                draw();
            }
        }

        private float[] gridX_lines =
        {
            10.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
            -10.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        };

        private float[] gridY_lines =
        {
            0.0f, 0.0f, 10.0f, 1.0f, 1.0f, 1.0f,
            0.0f, 0.0f, -10.0f, 1.0f, 1.0f, 1.0f,
        };

        Entity gridX, gridY, joints, path, goal, attr_points;
        Entity[] obstacles, boundings, attr_areas;
        List<Entity> tree = new List<Entity>();

        private static Shader _shader;
        
        private Camera _camera;
        private bool _firstMove = true;
        private Vector2 _lastPos;

        private double _time;
        
        private int Count = 0, MainCount = 0;
        private bool ShowAttractors, num1_prev;
        
        private Thread Main;
        private Attractor[] AttractorsLoc;

        public Window(int width, int height, string title) : base(width, height, GraphicsMode.Default, title) { }


        protected override void OnLoad(EventArgs e)
        {
            GL.ClearColor(0.2f, 0.3f, 0.3f, 1.0f);

            GL.Enable(EnableCap.DepthTest);

            _shader = new Shader(@"C:\Users\Dan\source\repos\R3T\Shaders\VertexShader.txt",
                @"C:\Users\Dan\source\repos\R3T\Shaders\FragmentShader.txt");
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

            // coordinate frame grid
            gridX = new Entity(gridX_lines);
            gridY = new Entity(gridY_lines);

            var input = Keyboard.GetState();
            num1_prev = input.IsKeyDown(Key.Number1);

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

            var input = Keyboard.GetState();
            if (input.IsKeyDown(Key.Number1) && !num1_prev)
            {
                ShowAttractors = !ShowAttractors;
                Console.WriteLine($"Times entered: {MainCount}");
            }

            //attractors' points
            if (attr_points == null)
            {
                if (Manager.States["Attractors"])
                {
                    Point[] points = new Point[Manager.Attractors.Count - 1];
                    for (int i = 0; i < points.Length; i++)
                    {
                        points[i] = Manager.Attractors[i + 1].Center;
                    }

                    attr_points = new Entity(GL_Convert(points, Vector3.UnitX));

                    AttractorsLoc = Manager.Attractors.GetRange(1, Manager.Attractors.Count - 1).ToArray();
                }
            }
            else if (ShowAttractors)
            {
                model = Matrix4.Identity;
                attr_points.Display(model, () =>
                {
                    GL.PointSize(5);
                    GL.DrawArrays(PrimitiveType.Points, 0, AttractorsLoc.Length);
                    GL.PointSize(1);
                });
            }

            //attractors' areas
            if (attr_areas == null)
            {
                if (Manager.States["Attractors"])
                {
                    Point[][] areas = new Point[Manager.Attractors.Count - 1][];
                    for (int i = 0; i < areas.Length; i++)
                    {
                        areas[i] = Manager.Attractors[i + 1].Area;
                    }

                    attr_areas = new Entity[Manager.Attractors.Count - 1];
                    
                    for (int i = 0; i < areas.Length; i++)
                    {
                        attr_areas[i] = new Entity(GL_Convert(areas[i], new Vector3(1, 0, 1)));
                    }
                }
            }
            else if (ShowAttractors)
            {
                model = Matrix4.Identity;
                for (int i = 0; i < attr_areas.Length; i++)
                {
                    attr_areas[i].Display(model, () =>
                    {
                        GL.DrawArrays(PrimitiveType.Points, 0, AttractorsLoc[i].Area.Length);
                    });
                }
            }

            // obstacles + boundings
            if (obstacles == null)
            {
                if (Manager.States["Obstacles"])
                {
                    obstacles = new Entity[Manager.Obstacles.Length];
                    boundings = new Entity[Manager.Obstacles.Length];
                    for (int i = 0; i < obstacles.Length; i++)
                    {
                        obstacles[i] = new Entity(GL_Convert(Manager.Obstacles[i].Data, Vector3.One));
                        boundings[i] = new Entity(GL_Convert(Manager.Obstacles[i].Bounding, Vector3.UnitY), new uint[]
                            {
                                0, 1, 2, 3, 0, 4, 5, 1, 5, 6, 2, 6, 7, 3, 7, 4
                            });
                    }
                }
            }
            else
            {
                model = Matrix4.Identity;
                for (int i = 0; i < obstacles.Length; i++)
                {
                    obstacles[i].Display(model, () =>
                    {
                        GL.DrawArrays(PrimitiveType.Points, 0, Manager.Obstacles[i].Data.Length);
                    });
                }
                
                for (int i = 0; i < boundings.Length; i++)
                {
                    boundings[i].Display(model, () =>
                    {
                        GL.DrawElements(BeginMode.LineStrip, 16, DrawElementsType.UnsignedInt, 0);
                    });
                }
            }

            // checking if the thread has aborted
            if (!Main.IsAlive)
            {
                // path
                if (path == null)
                {
                    if (Manager.States["Path"])
                        path = new Entity(GL_Convert(Manager.Path.ToArray(), Vector3.UnitX));
                }
                else
                {
                    model = Matrix4.Identity;
                    path.Display(model, () =>
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
                        joints = new Entity(GL_Convert(Manager.Joints[Count < Manager.Joints.Count - 1 ? Count++ : Count], new Vector3(1.0f, 0.5f, 0.0f)));

                    model = Matrix4.Identity;
                    joints.Display(model, () =>
                    {
                        GL.DrawArrays(PrimitiveType.LineStrip, 0, Manager.Manip.Links.Length + 1);
                    });
                }
            }

            //random tree
            if (Manager.Buffer.Count != 0)
            {
                for (int i = 0; i < Manager.Buffer.Count; i++)
                {
                    tree.Add(new Entity(GL_Convert(new Point[] { Manager.Buffer[i].p, Manager.Buffer[i].Parent.p }, Vector3.Zero)));
                }
                Manager.Buffer.Clear();
            }

            model = Matrix4.Identity;
            for (int i = 0; i < tree.Count; i++)
            {
                tree[i].Display(model, () =>
                {
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                });
            }

            // goal
            if (goal == null)
            {
                if (Manager.States["Goal"])
                {
                    List<Point> MainAttr = new List<Point> { Manager.Attractors[0].Center };
                    MainAttr.AddRange(Manager.Attractors[0].Area);
                    goal = new Entity(GL_Convert(MainAttr.ToArray(), new Vector3(1, 1, 0)));
                }
            }
            else
            {
                model = Matrix4.Identity;
                goal.Display(model, () =>
                {
                    GL.PointSize(5);
                    GL.DrawArrays(PrimitiveType.Points, 0, 1);
                    GL.PointSize(1);
                    GL.DrawArrays(PrimitiveType.Points, 1, Manager.Attractors[0].Area.Length);
                });
            }

            // X axis grid lines
            model = Matrix4.Identity;
            gridX.Display(model, () =>
            {
                GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
            });
            for (int i = 1; i < 11; i++)
            {
                model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitZ * i);
                gridX.Display(model, () =>
                {
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                });

                model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitZ * -i);
                gridX.Display(model, () =>
                {
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                });
            }

            // Y axis grid lines
            model = Matrix4.Identity;
            gridY.Display(model, () =>
            {
                GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
            });
            for (int i = 1; i < 11; i++)
            {
                model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitX * i);
                gridY.Display(model, () =>
                {
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                });

                model = Matrix4.Identity * Matrix4.CreateTranslation(Vector3.UnitX * -i);
                gridY.Display(model, () =>
                {
                    GL.DrawArrays(PrimitiveType.LineStrip, 0, 2);
                });
            }

            SwapBuffers();
            num1_prev = input.IsKeyDown(Key.Number1);

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
                _camera.Yaw += deltaX * sensitivity;
                _camera.Pitch -= deltaY * sensitivity; // reversed since y-coordinates range from bottom to top
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

        protected float[] GL_Convert(Point[] data, Vector3 color)
        {
            float[] res = new float[data.Length * 6];

            for (int i = 0; i < data.Length; i++)
            {
                res[6 * i] = (float)data[i].x;
                res[6 * i + 1] = (float)data[i].y;
                res[6 * i + 2] = (float)data[i].z;
                res[6 * i + 3] = color.X;
                res[6 * i + 4] = color.Y;
                res[6 * i + 5] = color.Z;
            }

            return res;
        }
    }
}