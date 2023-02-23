using BulletSharp;
using BulletSharp.SoftBody;
using Physics;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

public static class ArrayExtensions
{
    public static void Fill<T>(this T[] array, T value, int index = 0)
    {
        for (int i = index; i < array.Length; i++)
        {
            array[i] = value;
        }
    }

    public static int IndexOfMin<T>(this T[] array, Func<T, float> converter)
    {
        float min = converter(array[0]);
        int minIndex = 0;
        for (int i = 1; i < array.Length; i++)
        {
            float curr = converter(array[i]);
            if (curr < min)
            {
                min = curr;
                minIndex = i;
            }
        }

        return minIndex;
    }
}

public static class ListExtensions
{
    public static int IndexOfNearest<T>(this List<T> list, float value, Func<T, float> converter)
    {
        // return first/last if the value is outside the list
        if (value < converter(list[0]))
            return 0;

        if (value > converter(list[list.Count - 1]))
            return list.Count - 1;

        // if it's inside the list, perform binary search
        int low = 0;
        int high = list.Count - 1;
        while (low <= high)
        {
            int mid = (low + high) / 2;

            if (value < converter(list[mid]))
                high = mid - 1;
            else if (value > converter(list[mid]))
                low = mid + 1;
            else
                return mid;
        }

        return converter(list[low]) - value < value - converter(list[high]) ? low : high;
    }
}

public static class QueueExtensions
{
    public static void EnqueueBatch<T>(this Queue<T> queue, IEnumerable<T> batch)
    {
        var enumerator = batch.GetEnumerator();
        while (enumerator.MoveNext())
        {
            queue.Enqueue(enumerator.Current);
        }
    }

    public static IEnumerable<T> DequeueBatch<T>(this Queue<T> queue, int batchSize)
    {
        for (int i = 0; i < batchSize && queue.Count > 0; i++)
        {
            yield return queue.Dequeue();
        }
    }

    public static IEnumerable<T> DequeueAll<T>(this Queue<T> queue)
    {
        // snapshot of a current count maintains thread sync
        var count = queue.Count;
        for (int i = 0; i < count; i++)
        {
            yield return queue.Dequeue();
        }
    }
}

public static class ConcurrentQueueExtensions
{
    public static void EnqueueBatch<T>(this ConcurrentQueue<T> queue, IEnumerable<T> batch)
    {
        var enumerator = batch.GetEnumerator();
        while (enumerator.MoveNext())
        {
            queue.Enqueue(enumerator.Current);
        }
    }

    public static IEnumerable<T> DequeueAll<T>(this ConcurrentQueue<T> queue)  // TODO: define a maximum update rate (e.g. max 100 dequeues at once),
                                                                               // because otherwise dequeuing would cause freezes
    {
        var count = queue.Count;
        for (int i = 0; i < count; i++)
        {
            if (queue.TryDequeue(out T current))
            {
                yield return current;
            }
        }
    }
}

public static class Vector3Extensions
{
    public static float DistanceTo(this Vector3 v1, Vector3 v2)
    {
        return (v2 - v1).Length();
    }

    public static OpenTK.Mathematics.Vector3 ToOpenTK(this Vector3 vec)
    {
        return new OpenTK.Mathematics.Vector3(vec.X, vec.Y, vec.Z);
    }

    public static Vector3 Sum(this IEnumerable<Vector3> source)
    {
        return source.Aggregate((x, y) => x + y);
    }
}

public static class VectorConversionExtensions
{
    public static BulletSharp.Math.Vector4 ToBullet4(this OpenTK.Mathematics.Vector4 vec)
    {
        return new BulletSharp.Math.Vector4(vec.X, vec.Y, vec.Z, vec.W);
    }

    public static BulletSharp.Math.Vector3 ToBullet3(this OpenTK.Mathematics.Vector4 vec)
    {
        return new BulletSharp.Math.Vector3(vec.X, vec.Y, vec.Z);
    }

    public static BulletSharp.Math.Vector3 ToBullet3(this System.Numerics.Vector3 vec)
    {
        return new BulletSharp.Math.Vector3(vec.X, vec.Y, vec.Z);
    }

    public static System.Numerics.Vector4 ToNumerics4(this OpenTK.Mathematics.Vector4 vec)
    {
        return new Vector4(vec.X, vec.Y, vec.Z, vec.W);
    }

    public static System.Numerics.Vector3 ToNumerics3(this OpenTK.Mathematics.Vector4 vec)
    {
        return new Vector3(vec.X, vec.Y, vec.Z);
    }

    public static System.Numerics.Vector3 ToNumerics3(this OpenTK.Mathematics.Vector3 vec)
    {
        return new Vector3(vec.X, vec.Y, vec.Z);
    }

    public static System.Numerics.Vector3 ToNumerics3(this BulletSharp.Math.Vector3 vec)
    {
        return new Vector3(vec.X, vec.Y, vec.Z);
    }

    public static OpenTK.Mathematics.Vector3 ToOpenTK3(this BulletSharp.Math.Vector3 vec)
    {
        return new OpenTK.Mathematics.Vector3(vec.X, vec.Y, vec.Z);
    }

    public static System.Numerics.Vector2 ToNumerics2(this OpenTK.Mathematics.Vector2 vec)
    {
        return new Vector2(vec.X, vec.Y);
    }

    //public static System.Numerics.Vector3 XYZ(this System.Numerics.Vector4 vec)
    //{
    //    return new Vector3(vec.X, vec.Y, vec.Z);
    //}

    //public static System.Numerics.Vector2 XY(this System.Numerics.Vector3 vec)
    //{
    //    return new Vector2(vec.X, vec.Y);
    //}
}

public static class RigidBodyExtensions
{
    public static void SetType(this RigidBody body, RigidBodyType type)
    {
        switch (type)
        {
            case RigidBodyType.Static:
                body.CollisionFlags = CollisionFlags.StaticObject;
                body.ForceActivationState(ActivationState.ActiveTag);
                break;
            case RigidBodyType.Kinematic:
                body.CollisionFlags = CollisionFlags.StaticObject | CollisionFlags.KinematicObject;
                body.ForceActivationState(ActivationState.DisableDeactivation);
                break;
            case RigidBodyType.Dynamic:
                body.CollisionFlags = CollisionFlags.None;
                body.ForceActivationState(ActivationState.ActiveTag);
                body.Activate();
                break;
        }
    }
}

public static class VectorMathNetExtensions
{
    public static MathNet.Numerics.LinearAlgebra.Vector<float> AddSubVector(
        this MathNet.Numerics.LinearAlgebra.Vector<float> vec, 
        MathNet.Numerics.LinearAlgebra.Vector<float> sub)
    {
        for (int i = 0; i < sub.Count; i++)  // TODO: test performance
        {
            vec.At(i, vec.At(i) + sub.At(i));
        }

        return vec;
    }
}

public static class BulletSharpQuaternionExtensions
{
    public static BulletSharp.Math.Vector3 ToEuler(this BulletSharp.Math.Quaternion quat)
    {
        var eulers = new BulletSharp.Math.Vector3
        {
            X = (float)Math.Atan2(2 * (quat.W * quat.X + quat.Y * quat.Z), 1 - 2 * (quat.X * quat.X + quat.Y * quat.Y)),
            Z = (float)Math.Atan2(2 * (quat.W * quat.Z + quat.X * quat.Y), 1 - 2 * (quat.Y * quat.Y + quat.Z * quat.Z))
        };

        var expr = 2 * (quat.W * quat.Y - quat.Z * quat.X);
        if (Math.Abs(expr) >= 1)
            eulers.Y = (float)(Math.Sign(expr) * Math.PI / 2);
        else
            eulers.Y = (float)Math.Asin(expr);

        return eulers;
    }
}

public static class QuaternionExtensions
{
    public static Vector3 XYZ(this Quaternion quat)
    {
        return new Vector3(quat.X, quat.Y, quat.Z);
    }

    public static Vector3 Rotate(this Quaternion quat, Vector3 vec)
    {
        var xyz = new Vector3(quat.X, quat.Y, quat.Z);
        Vector3 uv = Vector3.Cross(xyz, vec);
        uv += uv;
        return vec + quat.W * uv + Vector3.Cross(xyz, uv);
    }

    public static OpenTK.Mathematics.Matrix3 ToMatrix(this Quaternion quat)
    {
        float w = quat.W, w2 = w * w;
        float x = quat.X, x2 = x * x;
        float y = quat.Y, y2 = y * y;
        float z = quat.Z, z2 = z * z;

        var rxx = w2 + x2 - y2 - z2;
        var rxy = 2 * x * y - 2 * w * z;
        var rxz = 2 * x * z + 2 * w * y;
        var ryx = 2 * x * y + 2 * w * z;
        var ryy = w2 - x2 + y2 - z2;
        var ryz = 2 * y * z - 2 * w * x;
        var rzx = 2 * x * z - 2 * w * y;
        var rzy = 2 * y * z + 2 * w * x;
        var rzz = w2 - x2 - y2 + z2;

        return new OpenTK.Mathematics.Matrix3(
            rxx, ryx, rzx,
            rxy, ryy, rzy,
            rxz, ryz, rzz
        );
    }
}

public static class MatrixExtensions
{
    public static OpenTK.Mathematics.Matrix4 TopOpenTK(this BulletSharp.Math.Matrix mat)
    {
        return new OpenTK.Mathematics.Matrix4(
            mat.M11, mat.M12, mat.M13, mat.M14,
            mat.M21, mat.M22, mat.M23, mat.M24,
            mat.M31, mat.M32, mat.M33, mat.M34,
            mat.M41, mat.M42, mat.M43, mat.M44
        );
    }

    public static BulletSharp.Math.Matrix ToBullet(this OpenTK.Mathematics.Matrix4 mat)
    {
        return new BulletSharp.Math.Matrix(
            mat.M11, mat.M12, mat.M13, mat.M14,
            mat.M21, mat.M22, mat.M23, mat.M24,
            mat.M31, mat.M32, mat.M33, mat.M34,
            mat.M41, mat.M42, mat.M43, mat.M44
        );
    }
}