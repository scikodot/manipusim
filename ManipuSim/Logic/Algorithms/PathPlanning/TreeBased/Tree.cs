using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;

using OpenTK.Graphics.OpenGL4;
using BulletSharp.Math;

using Logic.InverseKinematics;
using Graphics;

namespace Logic.PathPlanning
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public enum TreeBehaviour
    {
        Cyclic,
        Recursive
    }

    public class Tree : IDisposable
    {
        public class Node
        {
            public uint ID { get; set; }
            public Node Parent { get; set; }
            public List<Node> Childs { get; set; }

            public Vector3 Point { get; private set; }
            public VectorFloat q { get; private set; }

            public Node(Node parent, Vector3 point, VectorFloat q)
            {
                Parent = parent;
                Childs = new List<Node>();

                Point = point;
                this.q = q;
            }

            public override bool Equals(object obj)
            {
                if (obj is Node node)
                    return Point == node.Point;
                else
                    return false;
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    var hashCode = Point.X.GetHashCode();
                    hashCode = (hashCode * 397) ^ Point.Y.GetHashCode();
                    hashCode = (hashCode * 397) ^ Point.Z.GetHashCode();
                    return hashCode;
                }
            }
        }

        public class TreeModel : IDisposable
        {
            private readonly Queue<uint> _freeIndices = new Queue<uint>();
            private uint _freeTop;

            public Tree Tree { get; }
            public Model Model { get; }

            private Vector3 _color = Vector3.Zero;
            public ref Vector3 Color => ref _color;  // TODO: use in GUI!

            public bool IsSetup => Model.IsSetup;

            public TreeModel(Tree tree)
            {
                Tree = tree;

                // create an empty model with the specified max buffer size and material
                Model = new Model(new MeshVertex[tree.MaxSize], new uint[2 * (tree.MaxSize - 1)], 
                    new MeshMaterial { Diffuse = new OpenTK.Mathematics.Color4(_color.X, _color.Y, _color.Z, 1.0f) });
            }

            public void SetColor(Vector3 color)
            {
                Model.Meshes[0].Material = new MeshMaterial { Diffuse = new OpenTK.Mathematics.Color4(color.X, color.Y, color.Z, 1.0f) };
            }

            public void Render(Shader shader)
            {
                if (IsSetup)
                    Model.Render(shader, () =>
                    {
                        GL.DrawElements(BeginMode.Lines, 2 * ((int)_freeTop - 1), DrawElementsType.UnsignedInt, 0);
                    });
            }

            public void Reset()
            {
                Model.Meshes[0].UpdateVertices(0, Tree.MaxSize, new MeshVertex[Tree.MaxSize]);
                Model.Meshes[0].UpdateIndices(0, 2 * (Tree.MaxSize - 1), new uint[2 * (Tree.MaxSize - 1)]);

                _freeIndices.Clear();
                _freeTop = 0;
            }

            public void Update()
            {
                if (IsSetup)
                    AddNodes(Tree.AddBuffer.DequeueAll().ToList());
                    RemoveNodes(Tree.DelBuffer.DequeueAll().ToList());
            }

            private void AddNodes(List<Node> nodes)
            {
                foreach (var node in nodes)
                {
                    var point = node.Point;

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
                        Model.Meshes[0].UpdateIndices(2 * (node.ID - 1), 2, new uint[] { node.Parent.ID, node.ID });
                    }
                }
            }

            private void RemoveNodes(List<Node> nodes)
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

        public int MaxSize { get; }

        public HashSet<Node> Nodes { get; } = new HashSet<Node>();
        public ConcurrentQueue<Node> AddBuffer { get; } = new ConcurrentQueue<Node>();
        public ConcurrentQueue<Node> DelBuffer { get; } = new ConcurrentQueue<Node>();

        public TreeModel Model { get; }

        public TreeBehaviour Mode;

        public Node Root { get; private set; }
        public int Count => Nodes.Count;

        public Tree(Node root, int maxSize, TreeBehaviour mode = TreeBehaviour.Cyclic)
        {
            MaxSize = maxSize;

            Nodes.Add(root);

            Root = root;

            AddBuffer.Enqueue(root);

            Model = new TreeModel(this);

            Mode = mode;
        }

        public void AddNode(Node node)
        {
            // add the node to the base list
            Nodes.Add(node);

            // add the node to the parent's childs list
            node.Parent.Childs.Add(node);

            AddBuffer.Enqueue(node);
        }

        public void RemoveNode(Node node)
        {
            switch (Mode)
            {
                case TreeBehaviour.Cyclic:
                    var source = new Queue<Node>();
                    source.Enqueue(node);
                    while (source.Count != 0)
                    {
                        var nodeCurrent = source.Dequeue();

                        // remove the current node from the base list
                        Nodes.Remove(nodeCurrent);

                        // remove the current node from the parent's childs list
                        nodeCurrent.Parent.Childs.Remove(nodeCurrent);

                        // add childs of the current node to the deletion queue
                        source.EnqueueBatch(nodeCurrent.Childs);

                        DelBuffer.Enqueue(nodeCurrent);
                    }
                    break;
                case TreeBehaviour.Recursive:
                    RemoveNodeRecursive(node);
                    break;
            }
        }

        public void RemoveNodeRecursive(Node node)
        {
            // remove the node from the base list
            Nodes.Remove(node);

            // remove the node from the parent's childs list
            node.Parent.Childs.Remove(node);

            // remove all childs of the node
            var childs = new List<Node>(node.Childs);
            foreach (var child in childs)
            {
                RemoveNodeRecursive(child);
            }

            DelBuffer.Enqueue(node);
        }

        public Node Closest(Vector3 point)
        {
            return Nodes.MinBy(x => Vector3.Distance(x.Point, point));
        }

        public void Rectify()
        {
            Nodes.RemoveWhere(x => x.Childs.Count == 1);

            //Nodes.ForEach(x => x.Childs.Clear());

            var childsNew = new List<Node>[Nodes.Count];
            for (int i = 0; i < childsNew.Length; i++)
            {
                childsNew[i] = new List<Node>();
            }

            foreach (var node in Nodes)
            {
                var parent = node.Parent;
                while (parent.Childs.Count > 1 || parent != null)
                {
                    parent = parent.Parent;
                }
                node.Parent = parent;
            }

            // TODO: childs are not updated! fix
        }

        public void Trim(Manipulator manipulator, InverseKinematicsSolver solver)
        {
            switch (Mode)
            {
                case TreeBehaviour.Cyclic:
                    var source = new Queue<Node>();
                    source.Enqueue(Root);
                    while (source.Count != 0)
                    {
                        var childs = new List<Node>(source.Dequeue().Childs);
                        foreach (var child in childs)
                        {
                            manipulator.q = child.q;
                            if (manipulator.CollisionTest().Contains(true))
                            {
                                RemoveNode(child);
                            }
                            else
                            {
                                source.Enqueue(child);
                            }
                        }
                    }
                    break;
                case TreeBehaviour.Recursive:
                    foreach (var child in Root.Childs)
                    {
                        TrimRecursive(manipulator, solver, child);
                    }
                    break;
            }
        }

        public void TrimRecursive(Manipulator manipulator, InverseKinematicsSolver solver, Node node)
        {
            manipulator.q = node.q;
            if (manipulator.CollisionTest().Contains(true))
            {
                RemoveNode(node);
                return;
            }

            var childs = new List<Node>(node.Childs);
            foreach (var child in childs)
            {
                TrimRecursive(manipulator, solver, child);
            }
        }

        public Path GetPath(Manipulator manipulator, Node node)
        {
            var configs = GetConfigs(node);
            return new Path(configs.Select(config =>  // TODO: refactor! probably will have to add joints positions to tree nodes (same as for path)
            {
                manipulator.q = config;
                return manipulator.DKP;
            }), configs);
        }

        public IEnumerable<Vector3> GetPoints(Node node)
        {
            return TraversePoints(node).Reverse();
        }

        private IEnumerable<Vector3> TraversePoints(Node node)
        {
            while (node.Parent != null)
            {
                yield return node.Point;
                node = node.Parent;
            }
            yield return node.Point;
        }

        public IEnumerable<VectorFloat> GetConfigs(Node node)
        {
            return TraverseConfigs(node).Reverse();
        }

        private IEnumerable<VectorFloat> TraverseConfigs(Node node)
        {
            while (node.Parent != null)
            {
                yield return node.q;
                node = node.Parent;
            }
            yield return node.q;
        }

        public void Dispose()
        {
            Model.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}