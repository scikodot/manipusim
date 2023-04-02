using System;
using System.Collections.Generic;
using System.Threading.Tasks;

using BulletSharp;
using BulletSharp.Math;

using Graphics;
using Logic.InverseKinematics;
using Logic.PathPlanning;
using Physics;

namespace Logic
{
    public class ManipulatorHandler : IDisposable
    {
        private readonly MainWindow _parent;

        public int DefaultLinksNumber { get; set; } = 3;
        public float DefaultLinksLength { get; set; } = 1f;

        private Vector3 _defaultGoal = new Vector3(0.0f/*-1.0f*/, 0.5f, -2.0f);

        public List<Manipulator> Manipulators { get; } = new();

        public int Count => Manipulators.Count;

        public ManipulatorHandler(MainWindow parent)
        {
            _parent = parent;

            // pre-load models
            LoadDefaultModels();
        }

        private void LoadDefaultModels()
        {
            Joint.LoadDefaultModel(_parent.InputHandler.JointPath, _parent.InputHandler.GripperPath);
            Link.LoadDefaultModel(_parent.InputHandler.LinkPath);
        }

        public void Add(Manipulator manipulator)
        {
            Manipulators.Add(manipulator);
        }

        public void AddDefault(int linksNumber)
        {
            Add(Manipulator.CreateDefault(linksNumber));
        }

        public void Remove(Manipulator manipulator)
        {
            int index = Manipulators.IndexOf(manipulator);
            if (Manipulators.Remove(manipulator))
            {
                manipulator.Dispose();

                // remove goal model
                MainWindow._goalModels[index].Dispose();
                MainWindow._goalModels.RemoveAt(index);

                // remove goal model
                //MainWindow._treeModels[index].Dispose();
                //MainWindow._treeModels.RemoveAt(index);

                // remove goal model
                //MainWindow._pathModels[index].Dispose();
                //MainWindow._pathModels.RemoveAt(index);

                // remove goal model
                //MainWindow._gaModels[index].Dispose();
                //MainWindow._gaModels.RemoveAt(index);
            }
        }

        public void Update(InteractionMode mode)
        {
            foreach (var manipulator in Manipulators)
                manipulator.Update(mode);
        }

        public void OnInteractionModeSwitched(InteractionModeSwitchEventArgs e)
        {
            switch (e.Mode)
            {
                case InteractionMode.Design:
                    AbortControl();  // stop threads
                    Reset();  // reset positions
                    break;
                case InteractionMode.Simulate:
                    RunControl();  // start threads
                    break;
            }
        }

        public void RenderUnselected(ShaderProgram shader)
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.RenderUnselected(shader);

                if (manipulator.Path != null)
                    manipulator.Path.Model.Render(shader);

                if (manipulator.Controller.PathPlanner is RRT rrt)
                {
                    if (rrt.Tree != null)
                        rrt.Tree.Model.Render(shader);
                }
                else if (manipulator.Controller.PathPlanner is GeneticAlgorithm geneticAlgorithm)
                {
                    if (geneticAlgorithm.Dominant != null && geneticAlgorithm.Dominant.BezierCurve.Model != null)
                        geneticAlgorithm.Dominant.BezierCurve.Model.Render(shader);
                }
            }
        }

        public void RenderSelected(ShaderProgram shader)
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.RenderSelected(shader);
            }
        }

        public void Reset()
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.Reset();

                if (manipulator.Path != null)
                    manipulator.Path.Reset();

                if (manipulator.Controller.PathPlanner is RRT rrt)
                {
                    rrt.Tree.Model.Reset();
                }
                else if (manipulator.Controller.PathPlanner is GeneticAlgorithm geneticAlgorithm)
                {
                    geneticAlgorithm.Dominant = null;
                }
            }
        }

        private void RunControl()
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.Controller.Run();
            }
        }

        private void AbortControl()
        {
            foreach (var manipulator in Manipulators)
            {
                manipulator.Controller.Abort();
            }
        }

        public void Dispose()
        {
            // dispose of all the manipulators
            foreach (var manipulator in Manipulators)
            {
                manipulator.Dispose();

                // TODO: this should be included in a Manipulator disposal process
                if (manipulator.Controller.PathPlanner is RRT rrt)
                {
                    rrt.Tree.Dispose();
                }
            }
        }

        public static void Plan(Manipulator manip)
        {
            /*var resGA = PathPlanner.GeneticAlgorithm(manip, Obstacles, manip.Goal, resRRT.Item2.ToArray(), 
                0.99, manip.Joints.Length, 20, 0.95, 0.1, 10000, 
                PathPlanner.OptimizationCriterion.CollisionFree, 
                PathPlanner.SelectionMode.NormalDistribution, 
                PathPlanner.CrossoverMode.WeightedMean, 
                t => t * Math.PI / 180);*/

            /*var jac = new Jacobian(Obstacles, manip.q.Length, AD.Precision, AD.StepSize, AD.MaxTime);
            var resGA = PathPlanner.GeneticAlgorithmD(manip, Obstacles, manip.Goal, jac, 
                input, 0.99, manip.Joints.Length, 4, 0.95, 0.1, 10000,
                PathPlanner.OptimizationCriterion.CollisionFree,
                PathPlanner.SelectionMode.NormalDistribution,
                PathPlanner.CrossoverMode.WeightedMean,
                t => t * Math.PI / 180);*/
        }
    }
}
