using System;
using System.Collections.Generic;
using System.Linq;
using MoreLinq.Extensions;
using Graphics;
using Logic.InverseKinematics;

namespace Logic.PathPlanning
{
    public class Tree
    {
        public class Node  // cannot be a struct, because cyclic references (Node Parent) are not supported by structs
        {
            public Node Parent;
            public List<Node> Childs;

            public Vector3 Point;
            public Vector q;

            public Entity Entity;

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
        }

        public HashSet<Node> Nodes;
        public Queue<Node> AddBuffer, DelBuffer;

        public int Count => Nodes.Count;

        public Tree(Node root)
        {
            Nodes = new HashSet<Node> { root };

            AddBuffer = new Queue<Node>();
            DelBuffer = new Queue<Node>();
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
            // remove the node from the base list
            Nodes.Remove(node);

            // remove the node from the parent's childs list
            node.Parent.Childs.Remove(node);

            // remove all childs of the given node
            var childs = new List<Node>(node.Childs);
            foreach (var child in childs)
            {
                RemoveNode(child);
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

        public void Trim(Obstacle[] obstacles, Manipulator contestant, IKSolver solver, Node node)
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
                Trim(obstacles, contestant, solver, child);
            }
        }

        public void Trim(Obstacle[] obstacles, Manipulator contestant, IKSolver solver)
        {
            foreach (var child in Root.Childs)
            {
                Trim(obstacles, contestant, solver, child);
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
