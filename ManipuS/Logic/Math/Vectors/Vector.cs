using System;
using System.Collections.Generic;
using System.Linq;

namespace Logic
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public struct Vector
    {
        public float[] Components { get; private set; }

        public float this[int index]  // TODO: if making structs "readonly", this should be implemented in constructor
        {
            get => Components[index];
            set => Components[index] = value;
        }

        public int Size => Components.Length;

        public Vector(int size)
        {
            Components = new float[size];
        }

        public Vector(params float[] components)
        {
            Components = components;
        }

        public void Expand(int size)
        {
            Components = Components.Concat(new float[size]).ToArray();
        }

        public static float Dot(Vector v1, Vector v2)
        {
            return v1.Components.Zip(v2.Components, (x, y) => x * y).Sum();
        }

        public static Vector operator +(Vector v1, Vector v2)
        {
            return new Vector(v1.Components.Zip(v2.Components, (x, y) => x + y).ToArray());
        }

        public static Vector operator -(Vector v1, Vector v2)
        {
            return new Vector(v1.Components.Zip(v2.Components, (x, y) => x - y).ToArray());
        }

        public static Vector operator *(Matrix m, Vector v)  // TODO: optimize
        {
            float[] components = new float[m.RowsNumber];
            for (int i = 0; i < components.Length; i++)
            {
                components[i] = Dot(m.Rows[i], v);
            }

            return new Vector(components);
        }

        public static Vector operator *(Vector v, float s)
        {
            return new Vector(Array.ConvertAll(v.Components, x => x * s));
        }

        public static Vector operator *(float s, Vector v)
        {
            return new Vector(Array.ConvertAll(v.Components, x => x * s));
        }

        public static Vector operator /(Vector v, float s)
        {
            return new Vector(Array.ConvertAll(v.Components, x => x / s));
        }

        private static readonly string ListSeparator = System.Globalization.CultureInfo.CurrentCulture.TextInfo.ListSeparator;
        public override string ToString() => string.Format("({0})", string.Join($"{ListSeparator} ", Components.Select(x => string.Format("{0:0.###}", x))));
    }
}
