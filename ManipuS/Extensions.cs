using System;
using System.Collections.Generic;
using System.Linq;
using Logic;

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

public static class Vector3Extensions
{
    public static Vector3 Sum(this IEnumerable<Vector3> source)
    {
        return source.Aggregate((x, y) => x + y);
    }
}
