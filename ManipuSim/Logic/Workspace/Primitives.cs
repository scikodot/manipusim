using System;
using System.Collections.Generic;

using OpenTK.Mathematics;

using Graphics;
using BulletSharp;

namespace Logic
{
    // TODO: some methods return Model, while others return Mesh; 
    // presumably they should all return Mesh for flexibility, i.e. for composing Model of multiple meshes?
    // (see TranslationalWidget.Axis ctor)
    public static class Primitives  // TODO: refactor!!!
    {
        public static Model FromCollisionShape(CollisionShape shape, MeshMaterial material)
        {
            if (shape is BoxShape box)
            {
                var size = box.HalfExtentsWithMargin;
                return Cube(size.X, size.Y, size.Z, material);
            }
            else if (shape is SphereShape sphere)
            {
                var radius = sphere.Radius;
                return Sphere(radius, 20, 20, material);
            }
            // TODO: why does Primitives.Cylinder return a Mesh instead of a Model?
            else if (shape is CylinderShape cylinder)
            {
                var radius = cylinder.Radius;
                var halfLength = cylinder.HalfExtentsWithMargin.Y;
                return new Model(new Mesh[]
                {
                    Cylinder(radius, halfLength, halfLength, 20, material)
                });
            }
            // TODO: why does Primitives.Cone return a Mesh instead of a Model?
            else if (shape is ConeShape cone)
            {
                var radius = cone.Radius;
                var height = cone.Height;
                return new Model(new Mesh[]
                {
                    Cone(radius, height, 20, material)
                });
            }
            else
                throw new ArgumentException("Unknown collision shape.");
        }

        public static Model Grid(int lines, float stride, 
            MeshMaterial material, Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
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

            return new Model(vertices.ToArray(), indices, material, state, renderFlags: renderFlags);
        }

        public static Model Plane(float halfX, float halfY, 
            MeshMaterial material, Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
        {
            var vertices = new List<MeshVertex>();
            for (int z = -1; z <= 1; z += 2)
            {
                for (int x = -1; x <= 1; x += 2)
                {
                    vertices.Add(new MeshVertex { Position = new Vector3(x * halfX, 0.0f, z * halfY), Normal = Vector3.UnitY });
                }
            }

            var indices = new uint[]
            {
                0, 1, 2, 1, 2, 3
            };

            return new Model(vertices.ToArray(), indices, material, state, renderFlags: renderFlags);
        }

        public static Model Cube(float halfX, float halfY, float halfZ, 
            MeshMaterial material, Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
        {
            var size = new Vector3(halfX, halfY, halfZ);

            var vertices = new List<MeshVertex>();
            for (int z = -1; z <= 1; z += 2)
            {
                for (int y = -1; y <= 1; y += 2)
                {
                    for (int x = -1; x <= 1; x += 2)
                    {
                        var position = new Vector3(x, y, z) * size;
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

            return new Model(vertices.ToArray(), indices, material, state, renderFlags: renderFlags);
        }

        public static Model Sphere(float radius, uint stackCount, uint sectorCount, 
            MeshMaterial material, Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
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

            // return a model
            return new Model(vertices.ToArray(), indices.ToArray(), material, state, renderFlags: renderFlags);
        }

        public static Mesh Cylinder(float radius, float extentDown, float extentUp, int circleCount, 
            MeshMaterial material, Matrix4 state = default, RenderFlags renderFlags = RenderFlags.Solid)
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
                new MeshVertex { Position = new Vector3(0, -extentDown, 0), Normal = -Vector3.UnitY },
                new MeshVertex { Position = new Vector3(0, extentUp, 0), Normal = Vector3.UnitY }
            };

            float angle, cos, sin;
            for (int i = 0; i <= circleCount; i++)
            {
                angle = i * angleStep;
                cos = radius * (float)Math.Cos(angle);
                sin = radius * (float)Math.Sin(angle);

                // add LCi and UCi
                vertices.Add(new MeshVertex { Position = new Vector3(cos, -extentDown, sin), Normal = -Vector3.UnitY });
                vertices.Add(new MeshVertex { Position = new Vector3(cos, extentUp, sin), Normal = Vector3.UnitY });

                // add LSi and USi
                vertices.Add(new MeshVertex { Position = new Vector3(cos, -extentDown, sin), Normal = new Vector3(cos * radiusInv, 0, sin * radiusInv) });
                vertices.Add(new MeshVertex { Position = new Vector3(cos, extentUp, sin), Normal = new Vector3(cos * radiusInv, 0, sin * radiusInv) });
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

            //return new Model(vertices.ToArray(), indices.ToArray(), material, state, renderFlags: renderFlags);
            return new Mesh("Cylinder", vertices.ToArray(), indices.ToArray(), new MeshTexture[0], material);
        }

        public static Mesh Cone(float radius, float height, int circleCount, 
            MeshMaterial material, Vector3 origin = default/*, OpenTK.Matrix4 state = default*/, RenderFlags renderFlags = RenderFlags.Solid)
        {
            // cone center should be in the middle
            origin -= 0.5f * height * Vector3.UnitY;

            float radiusInv = 1.0f / radius;
            float angleStep = 2 * (float)Math.PI / circleCount;

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
            for (int i = 0; i <= circleCount; i++)
            {
                angle = i * angleStep;
                cos = radius * (float)Math.Cos(angle);
                sin = radius * (float)Math.Sin(angle);

                // add LCi
                vertices.Add(new MeshVertex { Position = origin + new Vector3(cos, 0, sin), Normal = -Vector3.UnitY });

                // add LSi
                vertices.Add(new MeshVertex { Position = origin + new Vector3(cos, 0, sin), Normal = new Vector3(cos * radiusInv, 0, sin * radiusInv) });
            }

            var indices = new List<uint>();

            // lower circle faces
            for (uint i = 0; i < circleCount; i++)
            {
                indices.Add(0);
                indices.Add(2 * i + 2);
                indices.Add(2 * i + 4);
            }

            // side faces
            for (uint i = 0; i < circleCount; i++)
            {
                indices.Add(1);
                indices.Add(2 * i + 3);
                indices.Add(2 * i + 5);
            }

            //return new Model(vertices.ToArray(), indices.ToArray(), material, state, renderFlags: renderFlags);
            return new Mesh("Cone", vertices.ToArray(), indices.ToArray(), new MeshTexture[0], material);
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
