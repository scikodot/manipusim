using System.Numerics;
using Graphics;
using OpenTK.Graphics.OpenGL4;

namespace Logic
{
    class BoxCollider : Collider
    {
        Vector3 Size;

        public BoxCollider(Vector3[] data) : base(data)
        {
            // retrieving boundary points
            float Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;
            Xmin = Xmax = data[0].X;
            Ymin = Ymax = data[0].Y;
            Zmin = Zmax = data[0].Z;
            for (int i = 0; i < data.Length; i++)
            {
                if (data[i].X < Xmin)
                    Xmin = data[i].X;
                else if (data[i].X > Xmax)
                    Xmax = data[i].X;

                if (data[i].Y < Ymin)
                    Ymin = data[i].Y;
                else if (data[i].Y > Ymax)
                    Ymax = data[i].Y;

                if (data[i].Z > Zmax)
                    Zmax = data[i].Z;
                else if (data[i].Z < Zmin)
                    Zmin = data[i].Z;
            }

            // data
            Data = new Vector3[8]
            {
                new Vector3(Xmin, Ymin, Zmin),
                new Vector3(Xmax, Ymin, Zmin),
                new Vector3(Xmax, Ymin, Zmax),
                new Vector3(Xmin, Ymin, Zmax),
                new Vector3(Xmin, Ymax, Zmin),
                new Vector3(Xmax, Ymax, Zmin),
                new Vector3(Xmax, Ymax, Zmax),
                new Vector3(Xmin, Ymax, Zmax)
            };

            Size = new Vector3(Xmax + Xmin, Ymax + Ymin, Zmax + Zmin);

            // central point
            Center = Size / 2;
        }

        public override bool Contains(Vector3 point)
        {
            return point.X >= Data[0].X && point.X <= Data[1].X &&
                   point.Y >= Data[0].Y && point.Y <= Data[4].Y &&
                   point.Z >= Data[0].Z && point.Z <= Data[2].Z;
        }

        public override Vector3 Extrude(Vector3 point)
        {
            Vector3 vec = point - Center;
            Vector3 diff = Size - vec;
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

        public override void Render(Shader shader, ref Matrix4 state)
        {
            if (Model == default)
                Model = new ComplexModel(Utils.GLConvert(Data), new uint[]
                {
                    0, 1, 2, 3, 0, 4, 5, 1, 5, 6, 2, 6, 7, 3, 7, 4
                }, MeshMaterial.Green);

            Model.State = state;
            Model.Render(shader, MeshMode.Solid, () =>
            {
                GL.DrawElements(BeginMode.LineStrip, 16, DrawElementsType.UnsignedInt, 0);
            });
        }
    }
}
