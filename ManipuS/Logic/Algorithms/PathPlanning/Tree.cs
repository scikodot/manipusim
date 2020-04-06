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
            public bool IsConnector;
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
        }

        public class Branch
        {
            public List<Node> Nodes;  // TODO: make private
            public List<Branch> Childs;  // TODO: make private

            public Node this[int index] => Nodes[index];

            public Node First => Nodes[0];
            public Node Last => Nodes[Nodes.Count - 1];

            public Branch(Node node)
            {
                Nodes = new List<Node> { node };
                Childs = new List<Branch>();

                node.IsConnector = true;
            }

            public void AddNode(Node node)
            {
                Nodes.Add(node);

                node.Branch = this;
            }

            public void RemoveNode(Node node)
            {
                Nodes.Remove(node);

                node.Branch = null;
            }

            public Branch AddBranch(Node node)
            {
                var branch = new Branch(node);
                Childs.Add(branch);
                return branch;
            }

            public void RemoveBranch(Branch branch)  // TODO: use somewhere
            {
                Childs.Remove(branch);
            }
        }

        //public List<List<Node>> Layers;  // TODO: create indexer, make layers private
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
            if (node.Parent.Childs.Count > 0)
            {
                var branch = node.Parent.Branch.AddBranch(node.Parent);
                branch.AddNode(node);
            }
            else
            {
                node.Parent.Branch.AddNode(node);
            }
            Count++;

            node.Parent.Childs.Add(node);

            AddBuffer.Enqueue(node);
        }

        public void RemoveNode(Node node)
        {
            //// delete node from layer
            //Layers[n.Layer].Remove(n);

            //// remove all childs of this node
            //foreach (var node in n.Childs)
            //    RemoveNode(node);

            //// delete current layer, if empty, with its successors
            //if (Layers[n.Layer].Count == 0)
            //    RemoveLayer(n.Layer);

            //Count--;

            //DelBuffer.Enqueue(n);

            node.Branch.RemoveNode(node);
            Count--;

            DelBuffer.Enqueue(node);
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
            foreach (var node in branch.Nodes)
            {
                contestant.q = node.q;
                if (solver.DetectCollisions(contestant, obstacles).Contains(true))
                {
                    // find the closest (upper) connector
                    Node nodeTarget = node;
                    while (!nodeTarget.IsConnector)
                        nodeTarget = nodeTarget.Parent;

                    // delete all the nodes after the found connector in the current branch
                    int indexNode = branch.Nodes.FindIndex(x => x == nodeTarget);
                    int countNode = branch.Nodes.Count - (indexNode + 1);
                    branch.Nodes.RemoveRange(indexNode + 1, countNode);  // TODO: explicitly release references of the nodes to the current branch
                    // TODO: also, after deleting all nodes, except the base node of the current branch, the branch remains alive; fix that
                    // TODO: explicitly delete nodes, so that difference is mirrored in add/del buffers
                    Count -= countNode;

                    // delete all the branches after the found connector
                    int indexBranch = branch.Childs.FindIndex(x => x.First == nodeTarget);
                    branch.Childs.RemoveRange(indexBranch + 1, branch.Childs.Count - (indexBranch + 1));

                    break;
                }
            }

            foreach (var br in branch.Childs)
                Trim(obstacles, contestant, solver, br);
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
