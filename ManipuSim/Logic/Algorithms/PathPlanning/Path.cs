using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;

using OpenTK.Graphics.OpenGL4;
using BulletSharp.Math;

using Graphics;

namespace Logic.PathPlanning
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public class Path : IDisposable
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

        public class PathModel : IDisposable
        {
            private static int _maxSize = 10000;
            public static ref int MaxSize => ref _maxSize;  // TODO: use in GUI!

            private readonly Queue<uint> _freeIndices = new Queue<uint>();
            private uint _freeTop;

            public Path Path { get; }
            public Model Model { get; }

            private Vector3 _color = Vector3.UnitX;
            public ref Vector3 Color => ref _color;  // TODO: use in GUI!

            public bool IsSetup => Model.IsSetup;

            public PathModel(Path path)
            {
                Path = path;

                // create an empty model with the specified material
                Model = new Model(new MeshVertex[_maxSize], new uint[2 * (_maxSize - 1)], 
                    new MeshMaterial { Diffuse = new OpenTK.Mathematics.Color4(_color.X, _color.Y, _color.Z, 1.0f) });
            }

            public void SetColor(Vector3 color)
            {
                Model.Meshes[0].Material = new MeshMaterial { Diffuse = new OpenTK.Mathematics.Color4(color.X, color.Y, color.Z, 1.0f) };
            }

            public void Render(ShaderProgram shader)
            {
                if (IsSetup)
                    Model.Render(shader, () =>
                    {
                        GL.DrawElements(BeginMode.Lines, 2 * ((int)_freeTop - 1), DrawElementsType.UnsignedInt, 0);
                    });
            }

            public void Reset()
            {
                Model.Meshes[0].UpdateVertices(0, _maxSize, new MeshVertex[_maxSize]);
                Model.Meshes[0].UpdateIndices(0, 2 * (_maxSize - 1), new uint[2 * (_maxSize - 1)]);

                _freeIndices.Clear();
                _freeTop = 0;
            }

            public void Update()
            {
                if (IsSetup)
                {
                    AddNodes(Path.AddBuffer.DequeueAll().ToList());
                    ChangeNodes(Path.ChgBuffer.DequeueAll().ToList());
                    RemoveNodes(Path.DelBuffer.DequeueAll().ToList());
                }
            }

            public void AddNodes(List<Node> nodes)
            {
                foreach (var node in nodes)
                {
                    var point = node.Points[node.Points.Length - 1];  // TODO: add property GripperPos to Path.Node

                    // add the node to the vertex buffer
                    uint index = _freeIndices.Count == 0 ? _freeTop++ : _freeIndices.Dequeue();
                    Model.Meshes[0].UpdateVertices(index, 1, new MeshVertex[]
                    {
                        new MeshVertex { Position = new OpenTK.Mathematics.Vector3(point.X, point.Y, point.Z) }
                    });

                    // memoize the index of the node for later use
                    node.ID = index;

                    if (node.Parent != null)
                    {
                        // add new indices pair to the element buffer
                        Model.Meshes[0].UpdateIndices(2 * (node.ID - 1), 2, new uint[2] { node.Parent.ID, node.ID });
                    }

                    if (node.Child != null)
                    {
                        // update child indices
                        Model.Meshes[0].UpdateIndices(2 * (node.Child.ID - 1), 1, new uint[1] { node.ID });
                    }
                }
            }

            public void ChangeNodes(List<Node> nodes)
            {
                foreach (var node in nodes)
                {
                    var point = node.Points[node.Points.Length - 1];  // TODO: add property GripperPos to Path.Node

                    // change the node point presented in the vertex buffer
                    Model.Meshes[0].UpdateVertices(node.ID, 1, new MeshVertex[]
                    {
                        new MeshVertex { Position = new OpenTK.Mathematics.Vector3(point.X, point.Y, point.Z) }
                    });
                }
            }

            public void RemoveNodes(List<Node> nodes)
            {
                // sort nodes list for sequential buffer filling
                nodes = nodes.OrderBy(x => x.ID).ToList();

                foreach (var node in nodes)
                {
                    // add the freed index to the list
                    _freeIndices.Enqueue(node.ID);

                    // remove the indices pair from the element buffer by setting it to all-zero
                    // TODO: this may cause performance damage if indices do not remove duplicate vertices; check!
                    Model.Meshes[0].UpdateIndices(2 * (node.ID - 1), 2, new uint[2] { 0, 0 });

                    if (node.Parent != null && node.Child != null)
                    {
                        // update child indices
                        Model.Meshes[0].UpdateIndices(2 * (node.Child.ID - 1), 1, new uint[1] { node.Parent.ID });
                    }
                }
            }

            public void Dispose()
            {
                // clear managed resources
                Model.Dispose();

                // suppress finalization
                GC.SuppressFinalize(this);
            }
        }

        public HashSet<Node> Nodes { get; } = new HashSet<Node>();
        public ConcurrentQueue<Node> AddBuffer { get; } = new ConcurrentQueue<Node>();
        public ConcurrentQueue<Node> DelBuffer { get; } = new ConcurrentQueue<Node>();
        public ConcurrentQueue<Node> ChgBuffer { get; } = new ConcurrentQueue<Node>();

        public PathModel Model { get; private set; }

        public Node First { get; private set; }
        public Node Last { get; private set; }
        public Node Current { get; private set; }
        public int Count => Nodes.Count;

        public Path(Node first)
        {
            AddNode(first);

            First = first;

            Current = First;

            Model = new PathModel(this);
        }

        public Path(IEnumerable<Vector3[]> points, IEnumerable<VectorFloat> configs, bool setModel = true) : 
            this(ConstructPath(points, configs), setModel) { }

        private static IEnumerable<Node> ConstructPath(IEnumerable<Vector3[]> points, IEnumerable<VectorFloat> configs)
        {
            Node parent = null;
            return points.Zip(configs, (p, c) =>
            {
                var current = new Node(parent, p, c);
                parent = current;
                return current;
            }).ToList();  // force evaluation to prevent references loss between nodes
        }

        private Path(IEnumerable<Node> nodes, bool setModel)
        {
            AddNodeRange(nodes);

            First = nodes.First();

            // on construction, current node is the first node
            Current = First;

            if (setModel)
                Model = new PathModel(this);
        }

        public void SetModel()
        {
            Model = new PathModel(this);
        }

        public IEnumerable<T> Select<T>(Func<Node, T> func)  // TODO: remove IEnumerables!
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
                length += Vector3.Distance(current.Points[lastIndex], current.Child.Points[lastIndex]);
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
            return new Path(DeepCopyNodes(), Model != null);
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

        public void Dispose()
        {
            Model.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
