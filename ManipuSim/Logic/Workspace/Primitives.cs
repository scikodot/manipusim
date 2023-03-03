using System;
using System.Collections.Generic;

using OpenTK.Mathematics;

using Graphics;
using BulletSharp;

namespace Logic
{
    // TODO: determine the purpose of _state_ param in mesh construction;
    // it can be used to set the initial displace of a mesh inside a model (that contains other meshes), 
    // but not sure that applying the transform matrix to each vertex is efficient
    public static class Primitives
    {
        public static Mesh FromCollisionShape(CollisionShape shape, MeshMaterial material)
        {
            return shape switch
            {
                BoxShape box => Cube(box.HalfExtentsWithMargin, material),
                SphereShape sphere => Sphere(sphere.Radius, 20, 20, material),
                CylinderShape cylinder => Cylinder(cylinder.Radius, cylinder.HalfExtentsWithMargin.Y, 20, material),
                ConeShape cone => Cone(cone.Radius, cone.Height, 20, material),
                _ => throw new ArgumentException("Unknown collision shape.")
            };
        }

        public static Mesh Grid(int lines, float stride, MeshMaterial material, Matrix4 transform = default)
        {
            float lineHalfLength = (lines - 1) * stride / 2;

            var vertices = new List<MeshVertex>();
            float current;
            for (int i = 0; i < lines; i++)
            {
                current = -lineHalfLength + i * stride;

                // X parallel lines
                vertices.Add(new MeshVertex { Position = new Vector3(-lineHalfLength, 0, current), Normal = Vector3.UnitY });
                vertices.Add(new MeshVertex { Position = new Vector3(lineHalfLength, 0, current), Normal = Vector3.UnitY });

                // Z parallel lines
                vertices.Add(new MeshVertex { Position = new Vector3(current, 0, -lineHalfLength), Normal = Vector3.UnitY });
                vertices.Add(new MeshVertex { Position = new Vector3(current, 0, lineHalfLength), Normal = Vector3.UnitY });
            }

            var indices = new uint[vertices.Count];
            for (uint i = 0; i < indices.Length; i++)
                indices[i] = i;

            return new Mesh(nameof(Grid), vertices.ToArray(), indices, Array.Empty<MeshTexture>(), material);
        }

        public static Mesh Plane(float halfExtentX, float halfExtentY, MeshMaterial material, Matrix4 transform = default)
        {
            var vertices = new List<MeshVertex>();
            for (int z = -1; z <= 1; z += 2)
            {
                for (int x = -1; x <= 1; x += 2)
                {
                    var position = new Vector3(x * halfExtentX, 0.0f, z * halfExtentY);
                    vertices.Add(new MeshVertex { Position = position, Normal = Vector3.UnitY });
                }
            }

            var indices = new uint[]
            {
                0, 1, 2, 1, 2, 3
            };

            return new Mesh(nameof(Plane), vertices.ToArray(), indices, Array.Empty<MeshTexture>(), material);
        }

        public static Mesh Cube(float halfExtent, MeshMaterial material, Matrix4 state = default) =>
            Cube(halfExtent, halfExtent, halfExtent, material, state);

        public static Mesh Cube(BulletSharp.Math.Vector3 size, MeshMaterial material, Matrix4 state = default) =>
            Cube(size.X, size.Y, size.Z, material, state);

        public static Mesh Cube(float halfExtentX, float halfExtentY, float halfExtentZ, MeshMaterial material, Matrix4 transform = default)
        {
            var vertices = new List<MeshVertex>();
            for (int z = -1; z <= 1; z += 2)
            {
                for (int y = -1; y <= 1; y += 2)
                {
                    for (int x = -1; x <= 1; x += 2)
                    {
                        var position = new Vector3(x * halfExtentX, y * halfExtentY, z * halfExtentZ);
                        vertices.Add(new MeshVertex { Position = position, Normal = x * Vector3.UnitX });
                        vertices.Add(new MeshVertex { Position = position, Normal = y * Vector3.UnitY });
                        vertices.Add(new MeshVertex { Position = position, Normal = z * Vector3.UnitZ });
                    }
                }
            }

            var indices = new uint[]
            {
                // X ortho faces
                0, 6, 12, 6, 12, 18,
                3, 9, 15, 9, 15, 21,

                // Y ortho faces
                1, 4, 13, 4, 13, 16,
                7, 10, 19, 10, 19, 22,

                // Z ortho faces
                2, 5, 8, 5, 8, 11,
                14, 17, 20, 17, 20, 23
            };

            return new Mesh(nameof(Cube), vertices.ToArray(), indices, Array.Empty<MeshTexture>(), material);
        }

        public static Mesh Sphere(float radius, uint stackCount, uint sectorCount, MeshMaterial material, Matrix4 transform = default)
        {
            float radiusInv = 1.0f / radius;
            float pi2 = (float)Math.PI / 2;
            float stackStep = (float)Math.PI / stackCount;
            float sectorStep = 2 * (float)Math.PI / sectorCount;

            var vertices = new List<MeshVertex>();
            float stackAngle, sectorAngle;
            float x, y, z, xy;  // positions
            //float s, t;  //textures
            for (int i = 0; i <= stackCount; i++)
            {
                stackAngle = pi2 - i * stackStep;
                xy = radius * (float)Math.Cos(stackAngle);
                z = radius * (float)Math.Sin(stackAngle);

                for (int j = 0; j <= sectorCount; j++)
                {
                    sectorAngle = j * sectorStep;
                    x = xy * (float)Math.Cos(sectorAngle);
                    y = xy * (float)Math.Sin(sectorAngle);

                    // textures
                    //s = (float)j / sectorCount;
                    //t = (float)i / stackCount;

                    var position = new Vector3(x, y, z);
                    vertices.Add(new MeshVertex
                    {
                        Position = position,
                        Normal = radiusInv * position
                    });
                }
            }

            var indices = new List<uint>();
            uint k1, k2;
            for (uint i = 0; i < stackCount; i++)
            {
                k1 = i * (sectorCount + 1);
                k2 = k1 + sectorCount + 1;

                for (int j = 0; j < sectorCount; j++, k1++, k2++)
                {
                    if (i != 0)
                    {
                        indices.Add(k1);
                        indices.Add(k2);
                        indices.Add(k1 + 1);
                    }

                    if (i != stackCount - 1)
                    {
                        indices.Add(k1 + 1);
                        indices.Add(k2);
                        indices.Add(k2 + 1);
                    }
                }
            }

            return new Mesh(nameof(Sphere), vertices.ToArray(), indices.ToArray(), Array.Empty<MeshTexture>(), material);
        }

        public static Mesh Cylinder(float radius, float halfLength, uint sectorCount, MeshMaterial material, Matrix4 transform = default)
        {
            var origin = transform.ExtractTranslation();

            float radiusInv = 1.0f / radius;
            float angleStep = 2 * (float)Math.PI / sectorCount;
            var heightVec = new Vector3(0, halfLength, 0);

            // the order of the vertices:
            // [L, U, LC1, UC1, LS1, US1, LC2, UC2, LS2, US2, ...],
            // where
            //      L - central point of the lower circle,
            //      U - central point of the upper circle,
            //      LCi - i-th point of the lower circle with central normal,
            //      UCi - i-th point of the upper circle with central normal,
            //      LSi - i-th point of the lower circle with side normal,
            //      USi - i-th point of the upper circle with side normal
            var vertices = new List<MeshVertex>
            {
                // add L and U
                new MeshVertex { Position = origin - heightVec, Normal = -Vector3.UnitY },
                new MeshVertex { Position = origin + heightVec, Normal = Vector3.UnitY }
            };

            float angle, cos, sin;
            for (int i = 0; i <= sectorCount; i++)
            {
                angle = i * angleStep;
                cos = radius * (float)Math.Cos(angle);
                sin = radius * (float)Math.Sin(angle);

                var radiusVec = new Vector3(cos, 0.0f, sin);
                vertices.Add(new MeshVertex { Position = origin - heightVec + radiusVec, Normal = -Vector3.UnitY });  // LCi
                vertices.Add(new MeshVertex { Position = origin + heightVec + radiusVec, Normal = Vector3.UnitY });  // UCi
                vertices.Add(new MeshVertex { Position = origin - heightVec + radiusVec, Normal = radiusInv * radiusVec });  // LSi
                vertices.Add(new MeshVertex { Position = origin + heightVec + radiusVec, Normal = radiusInv * radiusVec });  // USi
            }

            var indices = new List<uint>();

            // lower circle faces
            for (uint i = 0; i < sectorCount; i++)
            {
                indices.Add(0);
                indices.Add(4 * i + 2);
                indices.Add(4 * i + 6);
            }

            // upper circle faces
            for (uint i = 0; i < sectorCount; i++)
            {
                indices.Add(1);
                indices.Add(4 * i + 3);
                indices.Add(4 * i + 7);
            }

            // side faces
            for (uint i = 0; i < sectorCount; i++)
            {
                // lower triangle
                indices.Add(4 * i + 4);
                indices.Add(4 * i + 5);
                indices.Add(4 * i + 8);

                // upper triangle
                indices.Add(4 * i + 5);
                indices.Add(4 * i + 8);
                indices.Add(4 * i + 9);
            }

            return new Mesh(nameof(Cylinder), vertices.ToArray(), indices.ToArray(), Array.Empty<MeshTexture>(), material);
        }

        public static Mesh Cone(float radius, float height, uint sectorCount, MeshMaterial material, Matrix4 transform = default)
        {
            var origin = transform.ExtractTranslation();

            float radiusInv = 1.0f / radius;
            float angleStep = 2 * (float)Math.PI / sectorCount;

            // the order of the vertices:
            // [L, U, LC1, LS1, LC2, LS2, ...],
            // where
            //      L - central point of the lower circle,
            //      U - apex of the cone,
            //      LCi - i-th point of the lower circle with central normal,
            //      LSi - i-th point of the lower circle with side normal,
            var vertices = new List<MeshVertex>
            {
                // add L and U
                new MeshVertex { Position = origin, Normal = -Vector3.UnitY },
                new MeshVertex { Position = origin + height * Vector3.UnitY, Normal = Vector3.UnitY }
            };

            float angle, cos, sin;
            for (int i = 0; i <= sectorCount; i++)
            {
                angle = i * angleStep;
                cos = radius * (float)Math.Cos(angle);
                sin = radius * (float)Math.Sin(angle);

                var radiusVec = new Vector3(cos, 0.0f, sin);
                vertices.Add(new MeshVertex { Position = origin + radiusVec, Normal = -Vector3.UnitY });  // LCi
                vertices.Add(new MeshVertex { Position = origin + radiusVec, Normal = radiusInv * radiusVec }); // LSi
            }

            var indices = new List<uint>();

            // lower circle faces
            for (uint i = 0; i < sectorCount; i++)
            {
                indices.Add(0);
                indices.Add(2 * i + 2);
                indices.Add(2 * i + 4);
            }

            // side faces
            for (uint i = 0; i < sectorCount; i++)
            {
                indices.Add(1);
                indices.Add(2 * i + 3);
                indices.Add(2 * i + 5);
            }

            return new Mesh(nameof(Cone), vertices.ToArray(), indices.ToArray(), Array.Empty<MeshTexture>(), material);
        }

        /*public static System.Numerics.Vector3[] SpherePointCloud(float radius, System.Numerics.Vector3 center, int pointsNum)
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
        }*/
    }
}
