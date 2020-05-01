using Graphics;
using OpenTK;
using System;
using System.Collections.Generic;

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

        public static Model Cube(float halfX, float halfY, float halfZ, MeshMaterial material)
        {
            var model = new Model(cube, cubeIndices, material);
            model.State.M11 = 2 * halfX;
            model.State.M22 = 2 * halfY;
            model.State.M33 = 2 * halfZ;
            return model;
        }

        public static Model Sphere(float radius, int stackCount, int sectorCount, MeshMaterial material)
        {
            // get sphere vertices
            var vertices = new List<MeshVertex>();

            float x, y, z, xy;  // positions
            float nx, ny, nz;  // normals
            float s, t;  //textures
            float lengthInv = 1.0f / radius;

            float stackStep = (float)Math.PI / stackCount;
            float sectorStep = 2 * (float)Math.PI / sectorCount;
            float stackAngle, sectorAngle;
            for (int i = 0; i <= stackCount; i++)
            {
                stackAngle = (float)Math.PI / 2 - i * stackStep;
                xy = radius * (float)Math.Cos(stackAngle);
                z = radius * (float)Math.Sin(stackAngle);

                for (int j = 0; j <= sectorCount; j++)
                {
                    sectorAngle = j * sectorStep;

                    // positions
                    x = xy * (float)Math.Cos(sectorAngle);
                    y = xy * (float)Math.Sin(sectorAngle);

                    // normals
                    nx = x * lengthInv;
                    ny = y * lengthInv;
                    nz = z * lengthInv;

                    // textures
                    //s = (float)j / sectorCount;
                    //t = (float)i / stackCount;

                    vertices.Add(new MeshVertex
                    {
                        Position = new Vector3(x, y, z),
                        Normal = new Vector3(nx, ny, nz)
                    });
                }
            }

            // get sphere indices
            var indices = new List<uint>();

            int k1, k2;
            for (int i = 0; i < stackCount; i++)
            {
                k1 = i * (sectorCount + 1);
                k2 = k1 + sectorCount + 1;

                for (int j = 0; j < sectorCount; j++, k1++, k2++)
                {
                    if (i != 0)
                    {
                        indices.Add((uint)k1);
                        indices.Add((uint)k2);
                        indices.Add((uint)k1 + 1);
                    }

                    if (i != stackCount - 1)
                    {
                        indices.Add((uint)k1 + 1);
                        indices.Add((uint)k2);
                        indices.Add((uint)k2 + 1);
                    }
                }
            }

            // return a model
            return new Model(vertices.ToArray(), indices.ToArray(), material);
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
