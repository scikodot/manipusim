using Graphics;
using OpenTK;
using System;

namespace Logic
{
    public static class Primitives
    {
        private static readonly MeshVertex[] cube =
        {
            // X ortho faces
            new MeshVertex { Position = new Vector3(0.5f,  0.5f,  0.5f), Normal = new Vector3(1.0f, 0.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(0.5f,  0.5f, -0.5f), Normal = new Vector3(1.0f, 0.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(0.5f, -0.5f, 0.5f), Normal = new Vector3(1.0f, 0.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(0.5f, -0.5f, -0.5f), Normal = new Vector3(1.0f, 0.0f, 0.0f) },

            new MeshVertex { Position = new Vector3(-0.5f,  0.5f,  0.5f), Normal = new Vector3(-1.0f, 0.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-0.5f,  0.5f, -0.5f), Normal = new Vector3(-1.0f, 0.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-0.5f, -0.5f, 0.5f), Normal = new Vector3(-1.0f, 0.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-0.5f, -0.5f, -0.5f), Normal = new Vector3(-1.0f, 0.0f, 0.0f) },

            // Y ortho faces
            new MeshVertex { Position = new Vector3(0.5f,  0.5f, 0.5f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(0.5f,  0.5f, -0.5f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-0.5f,  0.5f,  0.5f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-0.5f,  0.5f,  -0.5f), Normal = new Vector3(0.0f, 1.0f, 0.0f) },

            new MeshVertex { Position = new Vector3(0.5f,  -0.5f, 0.5f), Normal = new Vector3(0.0f, -1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(0.5f,  -0.5f, -0.5f), Normal = new Vector3(0.0f, -1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-0.5f,  -0.5f,  0.5f), Normal = new Vector3(0.0f, -1.0f, 0.0f) },
            new MeshVertex { Position = new Vector3(-0.5f,  -0.5f,  -0.5f), Normal = new Vector3(0.0f, -1.0f, 0.0f) },

            // Z ortho faces
            new MeshVertex { Position = new Vector3(0.5f, 0.5f,  0.5f), Normal = new Vector3(0.0f, 0.0f, 1.0f) },
            new MeshVertex { Position = new Vector3(0.5f, -0.5f,  0.5f), Normal = new Vector3(0.0f, 0.0f, 1.0f) },
            new MeshVertex { Position = new Vector3(-0.5f,  0.5f,  0.5f), Normal = new Vector3(0.0f, 0.0f, 1.0f) },
            new MeshVertex { Position = new Vector3(-0.5f,  -0.5f,  0.5f), Normal = new Vector3(0.0f, 0.0f, 1.0f) },

            new MeshVertex { Position = new Vector3(0.5f, 0.5f,  -0.5f), Normal = new Vector3(0.0f, 0.0f, -1.0f) },
            new MeshVertex { Position = new Vector3(0.5f, -0.5f,  -0.5f), Normal = new Vector3(0.0f, 0.0f, -1.0f) },
            new MeshVertex { Position = new Vector3(-0.5f,  0.5f,  -0.5f), Normal = new Vector3(0.0f, 0.0f, -1.0f) },
            new MeshVertex { Position = new Vector3(-0.5f,  -0.5f,  -0.5f), Normal = new Vector3(0.0f, 0.0f, -1.0f) }
        };

        private static readonly uint[] cubeIndices =
        {
            // X ortho faces
            0, 1, 2,
            1, 2, 3,

            4, 5, 6,
            5, 6, 7,

            // Y ortho faces
            8, 9, 10,
            9, 10, 11,

            12, 13, 14,
            13, 14, 15,

            // Z ortho faces
            16, 17, 18,
            17, 18, 19,

            20, 21, 22,
            21, 22, 23
        };

        public static Model Cube(MeshMaterial material)
        {
            return new Model(cube, cubeIndices, material);
        }

        public static System.Numerics.Vector3[] SpherePointCloud(float radius, System.Numerics.Vector3 center, int pointsNum)
        {
            var data = new System.Numerics.Vector3[pointsNum];
            double x, yPos, y, zPos, z;
            for (int i = 0; i < pointsNum; i++)
            {
                x = radius * (2 * RandomThreadStatic.NextDouble() - 1);
                yPos = Math.Sqrt(radius * radius - x * x);
                y = yPos * (2 * RandomThreadStatic.NextDouble() - 1);
                zPos = Math.Sqrt(yPos * yPos - y * y);
                z = RandomThreadStatic.Next(0, 2) == 0 ? -zPos : zPos;

                data[i] = new System.Numerics.Vector3((float)x, (float)y, (float)z) + center;
            }

            return data;
        }
    }
}
