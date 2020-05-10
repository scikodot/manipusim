using Graphics;
using OpenTK;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Logic
{
    public static class Primitives  // TODO: perhaps unify?
    {
        public static Model Grid(int lines, float stride, MeshMaterial material, OpenTK.Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
        {
            float lineLengthHalf = (lines - 1) * stride / 2;

            var vertices = new List<MeshVertex>();
            for (int i = 0; i < lines; i++)
            {
                var current = -lineLengthHalf + i * stride;

                // Z parallel lines
                vertices.Add(new MeshVertex { Position = new Vector3(current, 0, -lineLengthHalf), Normal = Vector3.UnitY });
                vertices.Add(new MeshVertex { Position = new Vector3(current, 0, lineLengthHalf), Normal = Vector3.UnitY });

                // X parallel lines
                vertices.Add(new MeshVertex { Position = new Vector3(-lineLengthHalf, 0, current), Normal = Vector3.UnitY });
                vertices.Add(new MeshVertex { Position = new Vector3(lineLengthHalf, 0, current), Normal = Vector3.UnitY });
            }

            var indices = new uint[vertices.Count];
            for (uint i = 0; i < indices.Length; i++)
            {
                indices[i] = i;
            }

            return new Model(vertices.ToArray(), indices, material, state, renderFlags: renderFlags);
        }

        public static Model Plane(float width, float height, MeshMaterial material, OpenTK.Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
        {
            var vertices = new MeshVertex[]
            {
                new MeshVertex { Position = new Vector3(-width, 0.0f, -height), Normal = Vector3.UnitY },
                new MeshVertex { Position = new Vector3(width, 0.0f, -height), Normal = Vector3.UnitY },
                new MeshVertex { Position = new Vector3(-width, 0.0f, height), Normal = Vector3.UnitY },
                new MeshVertex { Position = new Vector3(width, 0.0f, height), Normal = Vector3.UnitY }
            };

            var indices = new uint[]
            {
                0, 1, 2, 1, 2, 3
            };

            return new Model(vertices, indices, material, state, renderFlags: renderFlags);
        }

        public static Model Cube(float halfX, float halfY, float halfZ, MeshMaterial material, OpenTK.Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
        {
            var vertices = new List<MeshVertex>();

            float x, y, z;
            foreach (var sign in new int[] { -1, 1 })
            {
                // X ortho faces
                x = sign * halfX;
                vertices.Add(new MeshVertex { Position = new Vector3(x, halfY, halfZ), Normal = new Vector3(sign, 0.0f, 0.0f) });
                vertices.Add(new MeshVertex { Position = new Vector3(x, halfY, -halfZ), Normal = new Vector3(sign, 0.0f, 0.0f) });
                vertices.Add(new MeshVertex { Position = new Vector3(x, -halfY, halfZ), Normal = new Vector3(sign, 0.0f, 0.0f) });
                vertices.Add(new MeshVertex { Position = new Vector3(x, -halfY, -halfZ), Normal = new Vector3(sign, 0.0f, 0.0f) });

                // Y ortho faces
                y = sign * halfY;
                vertices.Add(new MeshVertex { Position = new Vector3(halfX, y, halfZ), Normal = new Vector3(0.0f, sign, 0.0f) });
                vertices.Add(new MeshVertex { Position = new Vector3(halfX, y, -halfZ), Normal = new Vector3(0.0f, sign, 0.0f) });
                vertices.Add(new MeshVertex { Position = new Vector3(-halfX, y, halfZ), Normal = new Vector3(0.0f, sign, 0.0f) });
                vertices.Add(new MeshVertex { Position = new Vector3(-halfX, y, -halfZ), Normal = new Vector3(0.0f, sign, 0.0f) });

                // Z ortho faces
                z = sign * halfZ;
                vertices.Add(new MeshVertex { Position = new Vector3(halfX, halfY, z), Normal = new Vector3(0.0f, 0.0f, sign) });
                vertices.Add(new MeshVertex { Position = new Vector3(halfX, -halfY, z), Normal = new Vector3(0.0f, 0.0f, sign) });
                vertices.Add(new MeshVertex { Position = new Vector3(-halfX, halfY, z), Normal = new Vector3(0.0f, 0.0f, sign) });
                vertices.Add(new MeshVertex { Position = new Vector3(-halfX, -halfY, z), Normal = new Vector3(0.0f, 0.0f, sign) });
            }

            var indices = new List<uint>();

            uint k;
            for (uint i = 0; i < 6; i++)
            {
                k = 4 * i;

                // lower triangle
                indices.Add(k);
                indices.Add(k + 1);
                indices.Add(k + 2);

                // upper triangle
                indices.Add(k + 1);
                indices.Add(k + 2);
                indices.Add(k + 3);
            }

            return new Model(vertices.ToArray(), indices.ToArray(), material, state, renderFlags: renderFlags);
        }

        public static Model Sphere(float radius, uint stackCount, uint sectorCount, MeshMaterial material, OpenTK.Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
        {
            float radiusInv = 1.0f / radius;
            float pi2 = (float)Math.PI / 2;

            var vertices = new List<MeshVertex>();

            float x, y, z, xy;  // positions
            float nx, ny, nz;  // normals
            //float s, t;  //textures

            float stackStep = (float)Math.PI / stackCount;
            float sectorStep = 2 * (float)Math.PI / sectorCount;
            float stackAngle, sectorAngle;
            for (int i = 0; i <= stackCount; i++)
            {
                stackAngle = pi2 - i * stackStep;
                xy = radius * (float)Math.Cos(stackAngle);
                z = radius * (float)Math.Sin(stackAngle);

                for (int j = 0; j <= sectorCount; j++)
                {
                    sectorAngle = j * sectorStep;

                    // positions
                    x = xy * (float)Math.Cos(sectorAngle);
                    y = xy * (float)Math.Sin(sectorAngle);

                    // normals
                    nx = x * radiusInv;
                    ny = y * radiusInv;
                    nz = z * radiusInv;

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

            // return a model
            return new Model(vertices.ToArray(), indices.ToArray(), material, state, renderFlags: renderFlags);
        }

        public static Model Cylinder(float radius, float extendDown, float extendUp, int circleCount, MeshMaterial material, OpenTK.Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
        {
            float radiusInv = 1.0f / radius;
            float angleStep = 2 * (float)Math.PI / circleCount;

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
                new MeshVertex { Position = new Vector3(0, -extendDown, 0), Normal = -Vector3.UnitY },
                new MeshVertex { Position = new Vector3(0, extendUp, 0), Normal = Vector3.UnitY }
            };

            float angle, cos, sin;
            for (int i = 0; i <= circleCount; i++)
            {
                angle = i * angleStep;
                cos = radius * (float)Math.Cos(angle);
                sin = radius * (float)Math.Sin(angle);

                // add LCi and UCi
                vertices.Add(new MeshVertex { Position = new Vector3(cos, -extendDown, sin), Normal = -Vector3.UnitY });
                vertices.Add(new MeshVertex { Position = new Vector3(cos, extendUp, sin), Normal = Vector3.UnitY });

                // add LSi and USi
                vertices.Add(new MeshVertex { Position = new Vector3(cos, -extendDown, sin), Normal = new Vector3(cos * radiusInv, 0, sin * radiusInv) });
                vertices.Add(new MeshVertex { Position = new Vector3(cos, extendUp, sin), Normal = new Vector3(cos * radiusInv, 0, sin * radiusInv) });
            }

            var indices = new List<uint>();

            // lower circle faces
            for (uint i = 0; i < circleCount; i++)
            {
                indices.Add(0);
                indices.Add(4 * i + 2);
                indices.Add(4 * i + 6);
            }

            // upper circle faces
            for (uint i = 0; i < circleCount; i++)
            {
                indices.Add(1);
                indices.Add(4 * i + 3);
                indices.Add(4 * i + 7);
            }

            // side faces
            for (uint i = 0; i < circleCount; i++)
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

            return new Model(vertices.ToArray(), indices.ToArray(), material, state, renderFlags: renderFlags);
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
