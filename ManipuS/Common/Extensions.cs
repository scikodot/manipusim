using BulletSharp;
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

    public static OpenTK.Vector3 ToOpenTK(this Vector3 vec)
    {
        return new OpenTK.Vector3(vec.X, vec.Y, vec.Z);
    }

    public static Vector3 Sum(this IEnumerable<Vector3> source)
    {
        return source.Aggregate((x, y) => x + y);
    }
}

public static class VectorConversionExtensions
{
    public static BulletSharp.Math.Vector4 ToBullet4(this OpenTK.Vector4 vec)
    {
        return new BulletSharp.Math.Vector4(vec.X, vec.Y, vec.Z, vec.W);
    }

    public static BulletSharp.Math.Vector3 ToBullet3(this OpenTK.Vector4 vec)
    {
        return new BulletSharp.Math.Vector3(vec.X, vec.Y, vec.Z);
    }

    public static BulletSharp.Math.Vector3 ToBullet3(this System.Numerics.Vector3 vec)
    {
        return new BulletSharp.Math.Vector3(vec.X, vec.Y, vec.Z);
    }

    public static System.Numerics.Vector4 ToNumerics4(this OpenTK.Vector4 vec)
    {
        return new Vector4(vec.X, vec.Y, vec.Z, vec.W);
    }

    public static System.Numerics.Vector3 ToNumerics3(this OpenTK.Vector4 vec)
    {
        return new Vector3(vec.X, vec.Y, vec.Z);
    }

    public static System.Numerics.Vector3 ToNumerics3(this OpenTK.Vector3 vec)
    {
        return new Vector3(vec.X, vec.Y, vec.Z);
    }

    public static System.Numerics.Vector3 ToNumerics3(this BulletSharp.Math.Vector3 vec)
    {
        return new Vector3(vec.X, vec.Y, vec.Z);
    }

    public static OpenTK.Vector3 ToOpenTK3(this BulletSharp.Math.Vector3 vec)
    {
        return new OpenTK.Vector3(vec.X, vec.Y, vec.Z);
    }

    public static System.Numerics.Vector2 ToNumerics2(this OpenTK.Vector2 vec)
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
    public static void DisposeFromWorld(this RigidBody body)
    {
        Physics.PhysicsHandler.DisposeRigidBody(body);
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