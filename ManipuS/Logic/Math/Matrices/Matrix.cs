using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Logic
{
    public struct Matrix
    {
        public Vector[] Rows { get; }

        public float this[int row, int column] => Rows[row][column];

        public (int, int) Size => (RowsNumber, ColumnsNumber);

        public int RowsNumber => Rows.Length;

        public int ColumnsNumber => Rows[0].Size;

        public Matrix Transpose
        {
            get
            {
                var rows = new Vector[Rows[0].Size];
                for (int i = 0; i < rows.Length; i++)
                    rows[i] = new Vector(Rows.Select(x => x[i]).ToArray());

                return new Matrix(rows);
            }
        }

        public Matrix((int, int) size)
        {
            Rows = new Vector[size.Item1];
            for (int i = 0; i < Rows.Length; i++)
                Rows[i] = new Vector(size.Item2);
        }

        public Matrix(params Vector[] rows)
        {
            Rows = rows;
        }

        public static Matrix operator +(Matrix m1, Matrix m2)
        {
            return new Matrix(m1.Rows.Zip(m2.Rows, (x, y) => x + y).ToArray());
        }

        public static Matrix operator -(Matrix m1, Matrix m2)
        {
            return new Matrix(m1.Rows.Zip(m2.Rows, (x, y) => x - y).ToArray());
        }

        public static Matrix operator *(Matrix m, float s)
        {
            return new Matrix(Array.ConvertAll(m.Rows, x => x * s));
        }

        public static Matrix operator *(float s, Matrix m)
        {
            return new Matrix(Array.ConvertAll(m.Rows, x => x * s));
        }

        public static Matrix operator /(Matrix m, float s)
        {
            return new Matrix(Array.ConvertAll(m.Rows, x => x / s));
        }

        public override string ToString()
        {
            return string.Join("\n", Rows);
        }
    }
}
