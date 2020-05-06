using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.ComponentModel.Design.Serialization;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Runtime.Remoting.Messaging;
using System.Text;
using System.Threading.Tasks;

namespace Logic.PathPlanning
{
    public class Path
    {
        public class Node
        {
            public uint ID;
            public Node Parent, Child;

            public Vector3[] Points;
            public Vector q;

            public Node(Node parent, Vector3[] point, Vector q)
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

            public void Change(Vector3[] point, Vector config)
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

            //public Node ShallowCopy()
            //{
            //    return (Node)MemberwiseClone();
            //}
        }

        public HashSet<Node> Nodes;  // TODO: consider switching to List for maintaining order
        public ConcurrentQueue<Node> AddBuffer, DelBuffer;

        public Node First => Nodes.First();
        public Node Last { get; private set; }
        public int Count => Nodes.Count();

        public Path(IEnumerable<Vector3[]> points, IEnumerable<Vector> configs) : this(ConstructPath(points, configs)) { }

        private static IEnumerable<Node> ConstructPath(IEnumerable<Vector3[]> points, IEnumerable<Vector> configs)
        {
            Node parent = null;
            return points.Zip(configs, (p, c) =>
            {
                var current = new Node(parent, p, c);
                parent = current;
                return current;
            });
            //using (var pointsIter = points.GetEnumerator())
            //using (var configsIter = configs.GetEnumerator())
            //{
            //    Node parent = null;
            //    while (pointsIter.MoveNext() && configsIter.MoveNext())
            //    {
            //        var current = new Node(parent, pointsIter.Current, configsIter.Current);
            //        yield return current;
            //        parent = current;
            //    }
            //}
        }

        public Path(IEnumerable<Node> nodes)
        {
            Nodes = new HashSet<Node>(nodes);

            AddBuffer = new ConcurrentQueue<Node>();
            DelBuffer = new ConcurrentQueue<Node>();

            AddBuffer.EnqueueBatch(nodes);
        }

        public IEnumerable<T> Select<T>(Func<Node, T> func)
        {
            var current = First;
            while (current != null)
            {
                yield return func(current);
                current = current.Child;
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

        public void ChangeNode(Node node, Vector3[] point, Vector config)
        {
            DelBuffer.Enqueue(node);

            node.Change(point, config);

            AddBuffer.Enqueue(node);
        }
    }
}
