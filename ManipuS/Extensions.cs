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
}

public static class ListExtensions
{
    public static int NearestIndex<T>(this List<T> list, float value, Func<T, float> converter)
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
