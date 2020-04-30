using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Numerics;

using Logic.InverseKinematics;

using MoreLinq.Extensions;

namespace Logic.PathPlanning
{
    public enum TreeBehaviour
    {
        Cyclic = 0,
        Recursive = 1
    }

    public class Tree
    {
        public class Node  // cannot be a struct, because cyclic references (Node Parent) are not supported by structs
        {
            public Node Parent;
            public List<Node> Childs;

            public Vector3 Point;
            public Vector q;

            public Graphics.Model Model;

            public Node(Node parent, Vector3 point, Vector q)
            {
                Parent = parent;
                Childs = new List<Node>();

                Point = point;
                this.q = q;
            }

            public override bool Equals(object obj)
            {
                if (!(obj is Node))
                    return false;

                return Point == (obj as Node).Point;
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    var hashCode = Point.X.GetHashCode();
                    hashCode = (hashCode * 397) ^ Point.X.GetHashCode();
                    hashCode = (hashCode * 397) ^ Point.X.GetHashCode();
                    return hashCode;
                }
            }

            ~Node()
            {
                if (Model != default)
                {
                    Dispatcher.RenderActions.Enqueue(Model.Dispose);
                }
            }
        }

        public HashSet<Node> Nodes;
        public ConcurrentQueue<Node> AddBuffer, DelBuffer;
        public TreeBehaviour Mode;

        public int Count => Nodes.Count;

        public Tree(Node root, TreeBehaviour mode = TreeBehaviour.Cyclic)
        {
            Nodes = new HashSet<Node> { root };

            AddBuffer = new ConcurrentQueue<Node>();
            DelBuffer = new ConcurrentQueue<Node>();

            Mode = mode;
        }

        public Node Root => Nodes.First();

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

        public Node Min(Vector3 point)
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

        public void Trim(Obstacle[] obstacles, Manipulator contestant, IKSolver solver)
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

        public void TrimRecursive(Obstacle[] obstacles, Manipulator contestant, IKSolver solver, Node node)
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

        public IEnumerable<Vector3> TraversePath(Node node)
        {
            while (node.Parent != null)
            {
                yield return node.Point;
                node = node.Parent;
            }
            yield return node.Point;
        }

        public IEnumerable<Vector> TraverseConfigs(Node node)
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