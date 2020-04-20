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
                var rows = new Vector[ColumnsNumber];
                for (int i = 0; i < ColumnsNumber; i++)
                {
                    //rows[i] = new Vector(Rows.Length);
                    //for (int j = 0; j < Rows.Length; j++)
                    //{
                    //    rows[i][j] = Rows[j][i];
                    //}

                    rows[i] = new Vector(Rows.Select(x => x[i]).ToArray());
                }

                return new Matrix(rows);
            }
        }

        public Matrix(int rows, int columns)
        {
            Rows = new Vector[rows];
            for (int i = 0; i < Rows.Length; i++)
                Rows[i] = new Vector(columns);
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

        public static Matrix operator *(Matrix m1, Matrix m2)
        {
            var rows = new Vector[m1.RowsNumber];
            for (int i = 0; i < m1.RowsNumber; i++)
            {
                rows[i] = new Vector(m2.ColumnsNumber);
                for (int j = 0; j < m2.ColumnsNumber; j++)
                {
                    for (int k = 0; k < m1.ColumnsNumber; k++)
                    {
                        rows[i][j] += m1[i, k] * m2[k, j];
                    }
                }
            }

            return new Matrix(rows);
        }

        public override string ToString()
        {
            return string.Join("\n", Rows);
        }
    }
}
