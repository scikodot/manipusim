﻿using System.Collections.Generic;
using System.Linq;

using OpenTK.Graphics.OpenGL4;

using Logic.PathPlanning;
using Logic;
using System.Threading.Tasks;

namespace Graphics
{
    public class TreeModel
    {
        private readonly Queue<uint> _freeIndices = new Queue<uint>();
        private uint _freeTop;

        public Model Model { get; }

        public TreeModel(int maxSize, MeshMaterial material)
        {
            // create an empty model with the specified max buffer size and material
            Model = new Model(new MeshVertex[maxSize], new uint[2 * maxSize], material);
        }

        public void Render(Shader shader)
        {
            Model.Render(shader, () =>
            {
                GL.DrawElements(BeginMode.Lines, 2 * ((int)_freeTop - 1), DrawElementsType.UnsignedInt, 0);
            });
        }

        public void Reset()
        {
            Model.Meshes[0].UpdateVertices(0, 10000, new MeshVertex[10000]);
            Model.Meshes[0].UpdateIndices(0, 20000, new uint[20000]);
        }

        public void Update(int manipulatorNumber)
        {
            var toAdd = ManipHandler.Manipulators[manipulatorNumber].Tree.AddBuffer.DequeueAll().ToList();
            var toRemove = ManipHandler.Manipulators[manipulatorNumber].Tree.DelBuffer.DequeueAll().ToList();

            AddNodes(toAdd);
            RemoveNodes(toRemove);
        }

        private void AddNodes(List<Tree.Node> nodes)
        {
            foreach (var node in nodes)
            {
                var point = node.Point;

                // add the node to the vertex buffer
                uint index = _freeIndices.Count == 0 ? _freeTop++ : _freeIndices.Dequeue();
                Model.Meshes[0].UpdateVertices(index, 1, new MeshVertex[]
                {
                    new MeshVertex { Position = new OpenTK.Vector3(point.X, point.Y, point.Z) }
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

        private void RemoveNodes(List<Tree.Node> nodes)
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
    }
}