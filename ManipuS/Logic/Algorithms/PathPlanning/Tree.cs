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
            public Branch Branch;
            public int IndexOnTree;
            public bool IsConnector;

            public Vector3 Point;
            public Vector q;
            public Entity Entity;

            public int IndexOnBranch => IndexOnTree - Branch.First.IndexOnTree;
            public Node Previous => Parent;
            public Node Next => Branch.ElementAtOrDefault(IndexOnBranch + 1);

            public Node(Node parent, Vector3 point, Vector q)
            {
                Parent = parent;
                Point = point;
                this.q = q;
            }
        }

        public class Branch
        {
            public List<Node> Nodes;  // TODO: make private
            public List<Branch> Childs;  // TODO: make private
            public Entity Entity;

            public Node First => Nodes[0];
            public Node Last => Nodes[Nodes.Count - 1];
            public int Count => Nodes.Count;

            public Branch(Node node)
            {
                Nodes = new List<Node> { node };
                Childs = new List<Branch>();

                node.IsConnector = true;
            }

            public Node ElementAtOrDefault(int index)
            {
                return Nodes.ElementAtOrDefault(index);
            }

            public void AddNode(Node node)
            {
                Nodes.Add(node);

                node.Branch = this;
                node.IndexOnTree = node.Parent.IndexOnTree + 1;
            }

            public void RemoveNode(Node node)
            {
                Nodes.Remove(node);

                node.Branch = null;
                node.Parent = null;
            }

            public List<Node> Trim(Node node)
            {
                var removed = new List<Node>();
                var nodeCurr = Nodes[Nodes.Count - 1];
                while(nodeCurr != node)
                {
                    removed.Add(nodeCurr);
                    Nodes.RemoveAt(Nodes.Count - 1);
                    nodeCurr = nodeCurr.Parent;
                }

                while (!node.IsConnector)
                {
                    removed.Add(node);
                    Nodes.RemoveAt(Nodes.Count - 1);
                    node = node.Parent;
                }

                int branchIndex = Childs.FindLastIndex(x => x.First == node);

                for (int i = Childs.Count - 1; i > branchIndex; i--)
                {
                    removed.AddRange(Childs[i].Trim(Childs[i].First));
                }

                if (Nodes.Count == 1)
                    First.Branch.RemoveBranch(this);

                return removed;
            }

            public Branch AddBranch(Node node)
            {
                var branch = new Branch(node);
                Childs.Add(branch);
                Childs = Childs.OrderBy(x => x.First.IndexOnTree).ToList();
                return branch;
            }

            public void RemoveBranch(Branch branch)
            {
                Childs.Remove(branch);
            }
        }

        public Branch MainBranch;
        public Queue<Node> AddBuffer, DelBuffer;
        public int Count, LayersAdded;

        public Tree(Node root)
        {
            MainBranch = new Branch(root);
            Count = 1;

            root.Branch = MainBranch;

            AddBuffer = new Queue<Node>();
            DelBuffer = new Queue<Node>();
        }

        public Node Root => MainBranch.First;

        public void AddNode(Node node)
        {
            if (node.Parent.Next != null)
            {
                var branch = node.Parent.Branch.AddBranch(node.Parent);
                branch.AddNode(node);
            }
            else
            {
                node.Parent.Branch.AddNode(node);
            }
            Count++;

            AddBuffer.Enqueue(node);
        }

        public Node Min(Vector3 point)  // TODO: remove recursion!!! use cycles
        {
            return Min(point, MainBranch);
        }

        // TODO: optimize; this implementation has a complexity of (m + 2 * l + k), since it once more checks nodes, returned from child branches,
        // and nodes, where child branches connect to the current branch, hence adding the (2 * l) component;
        // meanwhile, it can have (m + k) by checking all nodes at once
        // m - amount of nodes on the current branch
        // l - amount of child branches of the current branch
        // k - total amount of nodes on the child branches
        public Node Min(Vector3 point, Branch branch)
        {
            var childsNodes = branch.Childs.Select(x => Min(point, x));

            return branch.Nodes.Concat(childsNodes).MinBy(x => x.Point.DistanceTo(point)).First();
        }

        public void Rectify(Branch branch)
        {
            foreach (var br in branch.Childs)
                Rectify(br);

            var connectors = new List<Node> { branch.First }.Concat(branch.Childs.Select(x => x.First)).Append(branch.Last).ToList();  // TODO: can be optimized

            // TODO: explicitly delete nodes, because:
            // 1) re-referencing does not introduce difference in tree structure, hence there's nothing to update in on-screen tree
            // 2) nodes on the shortcut do not get destroyed, they still remain in the branch nodes list
            for (int i = connectors.Count - 1; i > 0; i--)
                connectors[i].Parent = connectors[i - 1];
        }

        public void Rectify()
        {
            Rectify(MainBranch);
        }

        public void Trim(Obstacle[] obstacles, Manipulator contestant, IKSolver solver)
        {
            Trim(obstacles, contestant, solver, MainBranch);
        }

        public void Trim(Obstacle[] obstacles, Manipulator contestant, IKSolver solver, Branch branch)
        {
            var nodesCopy = new Node[branch.Nodes.Count];
            branch.Nodes.CopyTo(nodesCopy);
            for (int i = 0; i < branch.Nodes.Count; i++)
            {
                contestant.q = branch.Nodes[i].q;
                if (solver.DetectCollisions(contestant, obstacles).Contains(true))
                {
                    var removedNodes = branch.Trim(branch.Nodes[i]);
                    Count -= removedNodes.Count;

                    DelBuffer.EnqueueBatch(removedNodes);

                    break;
                }
            }

            for (int i = 0; i < branch.Childs.Count; i++)
                Trim(obstacles, contestant, solver, branch.Childs[i]);
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
