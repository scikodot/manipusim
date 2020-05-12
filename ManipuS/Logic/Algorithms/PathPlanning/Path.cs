using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Numerics;

namespace Logic.PathPlanning
{
    public class Path
    {
        public class Node
        {
            public uint ID { get; set; }
            public Node Parent { get; set; }
            public Node Child { get; set; }

            public Vector3[] Points { get; private set; }
            public MathNet.Numerics.LinearAlgebra.Vector<float> q { get; private set; }

            public Node(Node parent, Vector3[] point, MathNet.Numerics.LinearAlgebra.Vector<float> q)
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

                Points = point;
                this.q = q;
            }

            public void Change(Vector3[] point, MathNet.Numerics.LinearAlgebra.Vector<float> config)
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
        }

        public HashSet<Node> Nodes = new HashSet<Node>();
        public ConcurrentQueue<Node> AddBuffer = new ConcurrentQueue<Node>();
        public ConcurrentQueue<Node> DelBuffer = new ConcurrentQueue<Node>();

        public Node First { get; private set; }
        public Node Last { get; private set; }
        public Node Current { get; private set; }
        public int Count => Nodes.Count();

        public Path(IEnumerable<Vector3[]> points, IEnumerable<MathNet.Numerics.LinearAlgebra.Vector<float>> configs) : this(ConstructPath(points, configs)) { }

        private static IEnumerable<Node> ConstructPath(IEnumerable<Vector3[]> points, IEnumerable<MathNet.Numerics.LinearAlgebra.Vector<float>> configs)
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

        public IEnumerable<T> Select<T>(Func<Node, T> func)
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

        public void ChangeNode(Node node, Vector3[] point, MathNet.Numerics.LinearAlgebra.Vector<float> config)
        {
            DelBuffer.Enqueue(node);

            node.Change(point, config);

            AddBuffer.Enqueue(node);
        }

        public MathNet.Numerics.LinearAlgebra.Vector<float> Follow()
        {
            // move to the next node
            if (Current.Child != null)
                Current = Current.Child;

            // return the config of that node
            return Current.q;
        }
    }
}
