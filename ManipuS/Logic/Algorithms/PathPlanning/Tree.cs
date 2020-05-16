using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Numerics;

using Logic.InverseKinematics;

using MoreLinq.Extensions;

namespace Logic.PathPlanning
{
    using VectorFloat = MathNet.Numerics.LinearAlgebra.Vector<float>;

    public enum TreeBehaviour
    {
        Cyclic,
        Recursive
    }

    public class Tree
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

        public HashSet<Node> Nodes = new HashSet<Node>();
        public ConcurrentQueue<Node> AddBuffer = new ConcurrentQueue<Node>();
        public ConcurrentQueue<Node> DelBuffer = new ConcurrentQueue<Node>();
        public TreeBehaviour Mode;

        public Node Root { get; private set; }
        public int Count => Nodes.Count;

        public Tree(Node root, TreeBehaviour mode = TreeBehaviour.Cyclic)
        {
            Nodes.Add(root);

            Root = root;

            AddBuffer.Enqueue(root);

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
            return Nodes.MinBy(x => x.Point.DistanceTo(point)).First();
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

        public void Trim(Obstacle[] obstacles, Manipulator contestant, InverseKinematicsSolver solver)
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
                            contestant.q = child.q;
                            if (solver.DetectCollisions(contestant, obstacles).Contains(true))
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
                        TrimRecursive(obstacles, contestant, solver, child);
                    }
                    break;
            }
        }

        public void TrimRecursive(Obstacle[] obstacles, Manipulator contestant, InverseKinematicsSolver solver, Node node)
        {
            contestant.q = node.q;
            if (solver.DetectCollisions(contestant, obstacles).Contains(true))
            {
                RemoveNode(node);
                return;
            }

            var childs = new List<Node>(node.Childs);
            foreach (var child in childs)
            {
                TrimRecursive(obstacles, contestant, solver, child);
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
    }
}