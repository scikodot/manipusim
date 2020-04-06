using System;
using System.Collections.Generic;
using System.Linq;
using Graphics;

namespace Logic.PathPlanning
{
    public class Tree
    {
        public class Node  // cannot be a struct, because cyclic references (Node Parent) are not supported by structs
        {
            public Node Parent;
            public List<Node> Childs;
            public int Layer;
            public Vector3 Point;
            public Vector q;
            public Entity Entity;

            public Node(Node parent, Vector3 point, Vector q)
            {
                Parent = parent;
                Childs = new List<Node>();
                if (parent == null)
                    Layer = 0;
                else
                    Layer = parent.Layer + 1;
                Point = point;
                this.q = q;
            }
        }

        public List<List<Node>> Layers;  // TODO: create indexer, make layers private
        public Queue<Node> AddBuffer, DelBuffer;
        public int Count, LayersAdded;

        public Tree(Node root)
        {
            Layers = new List<List<Node>>
            {
                new List<Node>()
            };
            AddBuffer = new Queue<Node>();
            DelBuffer = new Queue<Node>();

            Layers[0].Add(root);
            Count = 1;
            LayersAdded = 0;
        }

        public Node Root => Layers[0][0];

        public void AddLayer()
        {
            Layers.Add(new List<Node>());
            LayersAdded++;
        }

        public void RemoveLayer(int layer)
        {
            for (int i = Layers.Count - 1; i >= layer; i--)
            {
                int layerCount = Layers[i].Count;
                Layers.RemoveAt(i);
                Count -= layerCount;
            }
        }

        public void RemoveNode(Node n)
        {
            // delete node from layer
            Layers[n.Layer].Remove(n);

            // remove all childs of this node
            foreach (var node in n.Childs)
                RemoveNode(node);

            // delete current layer, if empty, with its successors
            if (Layers[n.Layer].Count == 0)
                RemoveLayer(n.Layer);

            Count--;

            DelBuffer.Enqueue(n);
        }

        public void AddNode(Node n)
        {
            if (n.Layer == Layers.Count)
                AddLayer();

            n.Parent.Childs.Add(n);

            Layers[n.Layer].Add(n);
            Count++;

            AddBuffer.Enqueue(n);
        }

        public Node Min(Vector3 p)
        {
            Node min_node = null;
            float min = float.PositiveInfinity;
            foreach (var layer in Layers)
            {
                foreach (var node in layer)
                {
                    float curr = p.DistanceTo(node.Point);
                    if (curr < min)
                    {
                        min = curr;
                        min_node = node;
                    }
                }
            }
            return min_node;
        }

        public void Rectify(Node start)
        {
            Node node_curr = start.Parent;
            List<Node> nodes = new List<Node>();
            while (node_curr != null && node_curr.Parent != null && node_curr.Childs.Count == 1)
            {
                nodes.Add(node_curr);
                node_curr = node_curr.Parent;
            }

            if (nodes.Count > 0)
            {
                foreach (var node in nodes)
                {
                    RemoveNode(node);
                }
                start.Parent = node_curr;
            }
        }

        public void RectifyWhole()
        {
            for (int i = Layers.Count - 1; i >= 0; i--)
            {
                for (int j = 0; j < Layers[i].Count; j++)
                {
                    Rectify(Layers[i][j]);
                }
            }
        }

        public static Node[] Discretize(Node start, Node end, int pointNum)
        {
            Segment seg = new Segment(start.Point, end.Point);
            Vector3[] Vector3s = new Vector3[pointNum];
            Array.Copy(seg.Discretize(pointNum + 1), 1, Vector3s, 0, pointNum);

            Node parent, child;
            if (start.Layer > end.Layer)
            {
                parent = end;
                child = start;
            }
            else
            {
                parent = start;
                child = end;
            }

            Vector[] configs = new Vector[pointNum];
            for (int i = 0; i < pointNum; i++)
            {
                configs[i] = start.q + (i + 1) * (end.q - start.q) / (pointNum + 1);
            }

            Node[] nodes = new Node[pointNum];
            for (int i = 0; i < pointNum; i++)
            {
                nodes[i] = new Node(null, Vector3s[i], configs[i]);
            }
            return nodes;
        }
    }
}
