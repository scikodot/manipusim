using System;
using System.Linq;

namespace Logic
{
    public struct Vector
    {
        public float[] Components { get; private set; }

        public float this[int index] => Components[index];

        public float Length => (float)Math.Sqrt(Components.Sum(x => x * x));

        public float LengthSquared => Components.Sum(x => x * x);

        public Vector Normalized => this / Length;

        public Vector(int size)
        {
            Components = new float[size];
        }

        public Vector(float[] components)
        {
            this.Components = components;
        }

        public void Expand(int size)
        {
            Components = Components.Concat(new float[size]).ToArray();
        }

        public static Vector operator +(Vector v1, Vector v2)
        {
            return new Vector(v1.Components.Zip(v2.Components, (x, y) => x + y).ToArray());
        }

        public static Vector operator -(Vector v1, Vector v2)
        {
            return new Vector(v1.Components.Zip(v2.Components, (x, y) => x - y).ToArray());
        }

        public static Vector operator *(Vector v, float s)
        {
            return new Vector(Array.ConvertAll(v.Components, x => x * s));
        }

        public static Vector operator /(Vector v, float s)
        {
            return new Vector(Array.ConvertAll(v.Components, x => x / s));
        }

        public static Vector operator *(Matrix m, Vector v)  // TODO: optimize
        {
            float[] data = new float[m.Data.GetLength(0)];
            for (int i = 0; i < m.Data.GetLength(0); i++)
            {
                for (int j = 0; j < m.Data.GetLength(1); j++)
                {
                    data[i] += m[i, j] * v[j];
                }
            }

            return new Vector(data);
        }

        public override string ToString() => string.Format("[{0:2F}" + (Components.Length > 5 ? ", ...]" : "]"), string.Join(",", Components.Take(5)));
    }
}
