using System.Numerics;

using OpenTK.Graphics.OpenGL4;
using BulletSharp;

using Graphics;
using Physics;

namespace Logic
{
    class BoxCollider : ICollidable
    {
        private Vector3 _size;

        public Model Model { get; }
        public RigidBody Body { get; }

        //public BoxCollider(Vector3[] data)
        //{
        //    // retrieving boundary points
        //    float Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;
        //    Xmin = Xmax = data[0].X;
        //    Ymin = Ymax = data[0].Y;
        //    Zmin = Zmax = data[0].Z;
        //    for (int i = 0; i < data.Length; i++)
        //    {
        //        if (data[i].X < Xmin)
        //            Xmin = data[i].X;
        //        else if (data[i].X > Xmax)
        //            Xmax = data[i].X;

        //        if (data[i].Y < Ymin)
        //            Ymin = data[i].Y;
        //        else if (data[i].Y > Ymax)
        //            Ymax = data[i].Y;

        //        if (data[i].Z > Zmax)
        //            Zmax = data[i].Z;
        //        else if (data[i].Z < Zmin)
        //            Zmin = data[i].Z;
        //    }

        //    // data
        //    var vertices = new MeshVertex[8]
        //    {
        //        new MeshVertex { Position = new OpenTK.Vector3(Xmin, Ymin, Zmin) },
        //        new MeshVertex { Position = new OpenTK.Vector3(Xmax, Ymin, Zmin) },
        //        new MeshVertex { Position = new OpenTK.Vector3(Xmax, Ymin, Zmax) },
        //        new MeshVertex { Position = new OpenTK.Vector3(Xmin, Ymin, Zmax) },
        //        new MeshVertex { Position = new OpenTK.Vector3(Xmin, Ymax, Zmin) },
        //        new MeshVertex { Position = new OpenTK.Vector3(Xmax, Ymax, Zmin) },
        //        new MeshVertex { Position = new OpenTK.Vector3(Xmax, Ymax, Zmax) },
        //        new MeshVertex { Position = new OpenTK.Vector3(Xmin, Ymax, Zmax) }
        //    };

        //    Model = new Model(vertices, new uint[]
        //        {
        //            0, 1, 2, 3, 0, 4, 5, 1, 5, 6, 2, 6, 7, 3, 7, 4
        //        }, MeshMaterial.Green);
        //}

        public BoxCollider(float halfX, float halfY, float halfZ)
        {
            var vertices = new MeshVertex[8]
            {
                new MeshVertex { Position = new OpenTK.Vector3(-halfX, -halfY, -halfZ) },
                new MeshVertex { Position = new OpenTK.Vector3(halfX, -halfY, -halfZ) },
                new MeshVertex { Position = new OpenTK.Vector3(halfX, -halfY, halfZ) },
                new MeshVertex { Position = new OpenTK.Vector3(-halfX, -halfY, halfZ) },
                new MeshVertex { Position = new OpenTK.Vector3(-halfX, halfY, -halfZ) },
                new MeshVertex { Position = new OpenTK.Vector3(halfX, halfY, -halfZ) },
                new MeshVertex { Position = new OpenTK.Vector3(halfX, halfY, halfZ) },
                new MeshVertex { Position = new OpenTK.Vector3(-halfX, halfY, halfZ) }
            };

            Model = new Model(vertices, new uint[]
            {
                0, 1, 2, 3, 0, 4, 5, 1, 5, 6, 2, 6, 7, 3, 7, 4
            }, MeshMaterial.Green);

            // create a rigid body
            Body = PhysicsHandler.CreateKinematicBody(BulletSharp.Math.Matrix.Identity, new BoxShape(halfX, halfY, halfZ));

            _size = 2 * new Vector3(halfX, halfY, halfZ);
        }

        public bool Contains(Vector3 point)
        {
            var center = Body.CenterOfMassPosition;
            return point.X >= center.X - _size.X && point.X <= center.X + _size.X &&
                   point.Y >= center.Y - _size.Y && point.Y <= center.Y + _size.Y &&
                   point.Z >= center.Z - _size.Z && point.Z <= center.Z + _size.Z;
        }

        public Vector3 Extrude(Vector3 point)  // TODO: this method behaves weirdly; repair
        {
            var center = Body.CenterOfMassPosition;
            Vector3 vec = point - new Vector3(center.X, center.Y, center.Z);
            Vector3 diff = _size - vec;
            float ratio;
            if (diff.X > diff.Y)
            {
                if (diff.X > diff.Z)
                {
                    if (diff.Y > diff.Z)
                        ratio = diff.Z / vec.Z;
                    else
                        ratio = diff.Y / vec.Y;
                }
                else
                    ratio = diff.Y / vec.Y;
            }
            else
            {
                if (diff.Y > diff.Z)
                {
                    if (diff.X > diff.Z)
                        ratio = diff.Z / vec.Z;
                    else
                        ratio = diff.X / vec.X;
                }
                else
                    ratio = diff.X / vec.X;
            }

            return vec * ratio;
        }

        public void Render(Shader shader)
        {
            Model.Render(shader, MeshMode.Solid, () =>
            {
                GL.DrawElements(BeginMode.LineStrip, 16, DrawElementsType.UnsignedInt, 0);
            });
        }

        public void UpdateState(ref OpenTK.Matrix4 state)
        {
            Model.State = state;

            var stateMatrix = new BulletSharp.Math.Matrix(
                state.M11, state.M12, state.M13, state.M14,
                state.M21, state.M22, state.M23, state.M24,
                state.M31, state.M32, state.M33, state.M34,
                state.M41, state.M42, state.M43, state.M44);
            stateMatrix.Transpose();
            Body.MotionState.SetWorldTransform(ref stateMatrix);
        }
    }
}
