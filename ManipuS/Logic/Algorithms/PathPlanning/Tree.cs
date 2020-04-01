using System;
using System.Collections.Generic;
using Graphics;

namespace Logic.PathPlanning
{
    public class Tree
    {
        public class Node
        {
            public Entity Entity;

            public Node Parent;
            public List<Node> Childs;
            public int Layer;
            public Vector3 p;
            public float[] q;

            public Node(Node parent, Vector3 p, float[] q)
            {
                Parent = parent;
                Childs = new List<Node>();
                if (parent == null)
                    Layer = 0;
                else
                    Layer = parent.Layer + 1;
                this.p = p;
                this.q = Misc.CopyArray(q);
            }
        }

        public List<List<Node>> Layers;  // TODO: create indexer, make layers private
        public List<Node> AddBuffer;
        public List<Node> DelBuffer;
        public int Count, LayersAdded;

        public Tree(Node root)
        {
            Layers = new List<List<Node>>
            {
                new List<Node>()
            };
            AddBuffer = new List<Node>();
            DelBuffer = new List<Node>();

            Layers[0].Add(root);
            Count = 1;
            LayersAdded = 0;
        }

        public Tree(Tree tree)
        {

        }

        public Node Root
        {
            get { return Layers[0][0]; }
        }

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
            // waiting for thread to unlock
            Dispatcher.UpdateTree.WaitOne();

            // delete node from layer
            Layers[n.Layer].Remove(n);

            // remove all childs of this node
            foreach (var node in n.Childs)
                RemoveNode(node);

            // delete current layer, if empty, with its successors
            if (Layers[n.Layer].Count == 0)
                RemoveLayer(n.Layer);

            Count--;

            DelBuffer.Add(n);
        }

        public void AddNode(Node n)
        {
            // waiting for thread to unlock
            Dispatcher.UpdateTree.WaitOne();

            if (n.Layer == Layers.Count)
                AddLayer();

            n.Parent.Childs.Add(n);

            Layers[n.Layer].Add(n);
            Count++;

            AddBuffer.Add(n);
        }

        public Node Min(Vector3 p)
        {
            Node min_node = null;
            float min = float.PositiveInfinity;
            foreach (var layer in Layers)
            {
                foreach (var node in layer)
                {
                    float curr = p.DistanceTo(node.p);
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

        public static Node[] Discretize(Node start, Node end, int Vector3Num)
        {
            Segment seg = new Segment(start.p, end.p);
            Vector3[] Vector3s = new Vector3[Vector3Num];
            Array.Copy(seg.Discretize(Vector3Num + 1), 1, Vector3s, 0, Vector3Num);

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

            float[][] configs = new float[Vector3Num][];
            for (int i = 0; i < Vector3Num; i++)
            {
                float[] config = new float[start.q.Length];
                for (int j = 0; j < start.q.Length; j++)
                {
                    config[j] = start.q[j] + (i + 1) * (end.q[j] - start.q[j]) / (Vector3Num + 1);
                }
                configs[i] = config;
            }

            Node[] nodes = new Node[Vector3Num];
            for (int i = 0; i < Vector3Num; i++)
            {
                nodes[i] = new Node(null, Vector3s[i], configs[i]);
            }
            return nodes;
        }
    }
}
