//using System.Collections.Generic;
//using System.Linq;

//using OpenTK.Graphics.OpenGL4;

//using Logic.PathPlanning;
//using System;

//namespace Graphics
//{
//    public class PathModel : IDisposable
//    {
//        private readonly int _maxSize;
//        private readonly Queue<uint> _freeIndices = new Queue<uint>();
//        private uint _freeTop;

//        public Model Model { get; }
//        public bool IsSetup => Model.IsSetup;

//        public PathModel(int maxSize, MeshMaterial material)
//        {
//            // create an empty model with the specified max buffer size and material
//            _maxSize = maxSize;
//            Model = new Model(new MeshVertex[maxSize], new uint[2 * maxSize], material);
//        }

//        public void Render(Shader shader)
//        {
//            Model.Render(shader, () =>
//            {
//                GL.DrawElements(BeginMode.Lines, 2 * ((int)_freeTop - 1), DrawElementsType.UnsignedInt, 0);
//            });
//        }

//        public void Reset()
//        {
//            Model.Meshes[0].UpdateVertices(0, _maxSize, new MeshVertex[_maxSize]);
//            Model.Meshes[0].UpdateIndices(0, 2 * _maxSize, new uint[2 * _maxSize]);

//            _freeIndices.Clear();
//            _freeTop = 0;
//        }

//        public void Update(Path path)
//        {
//            var toAdd = path.AddBuffer.DequeueAll().ToList();
//            var toChange = path.ChgBuffer.DequeueAll().ToList();
//            var toRemove = path.DelBuffer.DequeueAll().ToList();

//            AddNodes(toAdd);
//            ChangeNodes(toChange);
//            RemoveNodes(toRemove);
//        }

//        public void AddNodes(List<Path.Node> nodes)
//        {
//            foreach (var node in nodes)
//            {
//                var point = node.Points[node.Points.Length - 1];  // TODO: add property GripperPos to Path.Node

//                // add the node to the vertex buffer
//                uint index = _freeIndices.Count == 0 ? _freeTop++ : _freeIndices.Dequeue();
//                Model.Meshes[0].UpdateVertices(index, 1, new MeshVertex[]
//                {
//                    new MeshVertex { Position = new OpenTK.Mathematics.Vector3(point.X, point.Y, point.Z) }
//                });

//                // memoize the index of the node for later use
//                node.ID = index;

//                if (node.Parent != null)
//                {
//                    // add new indices pair to the element buffer
//                    Model.Meshes[0].UpdateIndices(2 * (node.ID - 1), 2, new uint[2] { node.Parent.ID, node.ID });
//                }

//                if (node.Child != null)
//                {
//                    // update child indices
//                    Model.Meshes[0].UpdateIndices(2 * (node.Child.ID - 1), 1, new uint[1] { node.ID });
//                }
//            }
//        }

//        public void ChangeNodes(List<Path.Node> nodes)
//        {
//            foreach (var node in nodes)
//            {
//                var point = node.Points[node.Points.Length - 1];  // TODO: add property GripperPos to Path.Node

//                // change the node point presented in the vertex buffer
//                Model.Meshes[0].UpdateVertices(node.ID, 1, new MeshVertex[]
//                {
//                    new MeshVertex { Position = new OpenTK.Mathematics.Vector3(point.X, point.Y, point.Z) }
//                });
//            }
//        }

//        public void RemoveNodes(List<Path.Node> nodes)
//        {
//            // sort nodes list for sequential buffer filling
//            nodes = nodes.OrderBy(x => x.ID).ToList();

//            foreach (var node in nodes)
//            {
//                // add the freed index to the list
//                _freeIndices.Enqueue(node.ID);

//                // remove the indices pair from the element buffer by setting it to all-zero
//                // TODO: this may cause performance damage if indices do not remove duplicate vertices; check!
//                Model.Meshes[0].UpdateIndices(2 * (node.ID - 1), 2, new uint[2] { 0, 0 });

//                if (node.Parent != null && node.Child != null)
//                {
//                    // update child indices
//                    Model.Meshes[0].UpdateIndices(2 * (node.Child.ID - 1), 1, new uint[1] { node.Parent.ID });
//                }
//            }
//        }

//        public void Dispose()
//        {
//            // clear managed resources
//            Model.Dispose();

//            // suppress finalization
//            GC.SuppressFinalize(this);
//        }
//    }
//}
