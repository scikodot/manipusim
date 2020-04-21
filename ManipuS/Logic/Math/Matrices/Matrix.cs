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

        public float this[int row, int column]  // TODO: if making structs "readonly", this should be implemented in constructor
        {
            get => Rows[row][column];
            set => Rows[row][column] = value;
        }

        public (int, int) Size => (RowsNumber, ColumnsNumber);

        public int RowsNumber => Rows.Length;

        public int ColumnsNumber => Rows[0].Size;

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

        public Matrix Transpose()
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

        public float Determinant()
        {
            var lu = LU();

            // get determinant of the U matrix
            float detU = 1;
            for (int i = 0; i < RowsNumber; i++)
            {
                detU *= lu.Item3[i, i];
            }

            // detA = detP * detL * detU
            // -------------------------
            // detP is pre-calculated for convenience in LU decomposition
            // detL is 1 since L is a unitriangular matrix (all diagonal elements equal 1)
            // detU has to be calculated (see above)
            return lu.Item4 * detU;
        }

        public (Matrix, Matrix, Matrix, int) LU()
        {
            var size = RowsNumber;

            Matrix L = new Matrix(size, size);
            Matrix U = new Matrix(size, size);
            var pivot = Pivotize(this);

            // do optimized multiplication of the current matrix with its pivot
            var rows = new Vector[size];
            for (int i = 0; i < size; i++)
            {
                rows[i] = new Vector(size);
                for (int j = 0; j < size; j++)
                {
                    rows[i][j] = pivot.Item1[i, pivot.Item2[i]] * this[pivot.Item2[i], j];
                }
            }
            Matrix A2 = new Matrix(rows);  // TODO: this can be avoided

            for (int j = 0; j < size; j++)
            {
                L[j, j] = 1;
                for (int i = 0; i < j + 1; i++)
                {
                    float s1 = 0;
                    for (int k = 0; k < i; k++)
                        s1 += U[k, j] * L[i, k];
                    U[i, j] = A2[i, j] - s1;
                }
                for (int i = j; i < size; i++)
                {
                    float s2 = 0;
                    for (int k = 0; k < j; k++)
                        s2 += U[k, j] * L[i, k];

                    // if U has zero diagonal elements even after pivoting, the determinant is zero, hence no need to calculate further
                    if (U[j, j] == 0)  // TODO: maybe there's a better way to check?
                        goto ZeroDet;
                    L[i, j] = (A2[i, j] - s2) / U[j, j];
                }
            }
            ZeroDet:

            return (pivot.Item1, L, U, pivot.Item3);
        }

        private static (Matrix, int[], int) Pivotize(Matrix mat)
        {
            var size = mat.ColumnsNumber;
            Vector[] columns = new Vector[size];

            int determinant = 1;
            int[] indices = new int[size];
            for (int i = 0; i < size; i++)
            {
                float max = mat[i, i];
                int row = i;

                for (int j = i; j < size; j++)
                {
                    if (mat[j, i] > max)
                    {
                        max = mat[j, i];
                        row = j;
                    }
                }

                Vector column = new Vector(size);
                column[i] = 1;
                columns[row] = column;

                indices[i] = row;

                if (i != row)
                {
                    // the columns were swapped, hence increase swap count
                    determinant = -determinant;
                }
            }

            return (new Matrix(columns), indices, determinant);
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
