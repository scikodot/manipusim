using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Numerics;

namespace Logic.PathPlanning
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public class Path
    {
        public class Node
        {
            public uint ID { get; set; }
            public Node Parent { get; set; }
            public Node Child { get; set; }

            public Vector3[] Points { get; set; }
            public VectorFloat q { get; set; }  // TODO: for unified usage of path consider replacing with "object Context" or something similar

            public Node(Node parent, Vector3[] points, VectorFloat q)
            {
                // assign the parent to the current node's parent
                Parent = parent;

                if (parent != null)
                {
                    if (parent.Child != null)
                    {
                        // assign the parent's child to the current node's child
                        Child = parent.Child;

                        // assign the current node to the child's parent
                        Child.Parent = this;
                    }

                    // assign the current node to the parent's child
                    parent.Child = this;
                }

                Points = points;
                this.q = q;
            }

            public void Change(Vector3[] point, VectorFloat config)
            {
                Points = point;
                q = config;
            }

            public override bool Equals(object obj)
            {
                if (!(obj is Node))
                    return false;

                return Points == (obj as Node).Points;
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    var hashCode = Points[0].X.GetHashCode();
                    hashCode = (hashCode * 397) ^ Points[0].Y.GetHashCode();
                    hashCode = (hashCode * 397) ^ Points[0].Z.GetHashCode();
                    return hashCode;
                }
            }

            public Node ShallowCopy()
            {
                var node = (Node)MemberwiseClone();

                q.CopyTo(node.q);

                return node;
            }
        }

        public HashSet<Node> Nodes = new HashSet<Node>();
        public ConcurrentQueue<Node> AddBuffer = new ConcurrentQueue<Node>();
        public ConcurrentQueue<Node> DelBuffer = new ConcurrentQueue<Node>();
        public ConcurrentQueue<Node> ChgBuffer = new ConcurrentQueue<Node>();

        public Node First { get; private set; }
        public Node Last { get; private set; }
        public Node Current { get; private set; }
        public int Count => Nodes.Count();

        public Path(Node first)
        {
            AddNode(first);

            First = first;

            Current = First;
        }

        public Path(IEnumerable<Vector3[]> points, IEnumerable<VectorFloat> configs) : this(ConstructPath(points, configs)) { }

        private static IEnumerable<Node> ConstructPath(IEnumerable<Vector3[]> points, IEnumerable<VectorFloat> configs)
        {
            Node parent = null;
            return points.Zip(configs, (p, c) =>
            {
                var current = new Node(parent, p, c);
                parent = current;
                return current;
            }).ToList();  // force evaluation to prevent losing references between nodes
        }

        private Path(IEnumerable<Node> nodes)
        {
            AddNodeRange(nodes);

            First = nodes.First();

            // on construction, current node is the first node
            Current = First;
        }

        public IEnumerable<T> Select<T>(Func<Node, T> func)  // TODO: remove IEnumerables!
        {
            var current = First;
            while (current != null)
            {
                yield return func(current);
                current = current.Child;  // TODO: might be not executed; check
            }
        }

        public void AddNode(Node node)
        {
            // add the node to the base list
            Nodes.Add(node);

            // if the current node does not have child, it is the last node in the path
            if (node.Child == null)
                Last = node;

            AddBuffer.Enqueue(node);
        }

        public void AddLast(Vector3[] points, VectorFloat configuration)
        {
            AddNode(new Node(Last, points, configuration));
        }

        public void AddNodeRange(IEnumerable<Node> nodes)
        {
            foreach (var node in nodes)
            {
                AddNode(node);
            }
        }

        public void RemoveNode(Node node)
        {
            // remove the current node from the base list
            Nodes.Remove(node);

            // remove the current node from the child's parent
            node.Child.Parent = node.Parent;

            // remove the current node from the parent's child
            node.Parent.Child = node.Child;

            DelBuffer.Enqueue(node);
        }

        public void ChangeNode(Node node, Vector3[] points, VectorFloat config)
        {
            node.Change(points, config);
            ChgBuffer.Enqueue(node);
        }

        public Node Follow()
        {
            // move to the next node
            if (Current.Child != null)
                Current = Current.Child;

            // return that node
            return Current;
        }

        public float GetLength()
        {
            float length = 0;
            Node current = First;
            int lastIndex = current.Points.Length - 1;
            while (current.Child != null)
            {
                length += current.Points[lastIndex].DistanceTo(current.Child.Points[lastIndex]);
                current = current.Child;
            }

            return length;
        }

        public void Reset()
        {
            Current = First;
        }

        public void Translate(Vector3 translation)
        {
            Node current = First;
            while (current != null)
            {
                ChangeNode(current, new Vector3[] { current.Points[0] + translation }, current.q);
                current = current.Child;
            }
        }

        //public IEnumerable<Vector3> GetGripperPath()
        //{
        //    var current = First;
        //    while (current != null)
        //    {
        //        yield return current.Points[current.Points.Length - 1];
        //        current = current.Child;  // TODO: might be not executed; check
        //    }
        //}

        public Path DeepCopy()
        {
            return new Path(DeepCopyNodes());
        }

        private List<Node> DeepCopyNodes()
        {
            var nodes = new List<Node>();

            var current = First;
            Node previous = null;
            while (current != null)
            {
                current = current.ShallowCopy();

                if (previous != null)
                {
                    previous.Child = current;
                    current.Parent = previous;
                }

                nodes.Add(current);

                previous = current;
                current = current.Child;

                //nodes.Add(current);
                //current = current.Child;
            }

            return nodes;
        }
    }
}
