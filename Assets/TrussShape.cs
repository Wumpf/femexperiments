using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEditor;
using UnityEngine;
using UnityEngine.VR;
using UnityEngine.VR.WSA;

// A 2D structure whose line elements can experience only axial stress
public class TrussShape : MonoBehaviour
{
    [Serializable]
    public struct Node
    {
        public Node(Vector2 pos)
        {
            Position = pos;
            Fixed = false;
        }
        
        public Vector2 Position;
        /// <summary>
        /// If true, the node is constrained, i.e. it cannot be moved.
        /// </summary>
        public bool Fixed;
    }

    [Serializable]
    public class Element
    {
        [Tooltip("Area in square meter.")]
        public float CrossSectionalArea = 0.01f;    // For a rod: Thickness * Thickness * (float) Math.PI;
        [Tooltip("Describes stiffness of material. Rubber has 0.01-0.1, stell has 209")]
        public float YoungModulusGPa = 1.0f;

        public float YoungModulus => YoungModulusGPa * 1000000000;

        [Tooltip("kg per cubic meter. 1000 is water.")]
        public float Density = 1000.0f;

        public float DampingCoefficient = 50.0f;
        
        public int LeftNodeIdx;
        public int RightNodeIdx;

        public Element(int leftNodeIdx, int rightNodeIdx)
        {
            LeftNodeIdx = leftNodeIdx;
            RightNodeIdx = rightNodeIdx;
        }

        /// <summary>
        /// Rotation matrix that is used to convert from 1D bar element to the 2D truss element
        /// </summary>
        /// <param name="dir">Normalixed from one node to the other.</param>
        private Matrix<float> GetRotationMatrix(Vector2 dir)
        {
            float cosAngle = dir.x; // dot product with x axis divided by length.
            float sinAngle = (float)Math.Sqrt(1.0f - cosAngle * cosAngle); // http://www.wolframalpha.com/input/?i=sin(cos%CB%86-1(x))
            return DenseMatrix.OfArray(new[,] {{cosAngle, 0.0f}, {sinAngle, 0.0f}, {0.0f, cosAngle}, {0.0f, sinAngle}});
        }

        private void GetDirAndLength(IList<Node> nodes, out Vector2 dir, out float length)
        {
            dir = nodes[RightNodeIdx].Position - nodes[LeftNodeIdx].Position;
            length = dir.magnitude;
            dir /= length; 
        }
        
        public Matrix<float> GetStiffnessMatrix(IList<Node> nodes)
        {
            Vector2 dir;
            float length;
            GetDirAndLength(nodes, out dir, out length);
            
            // using a linear shape function
            float k = CrossSectionalArea * YoungModulus / length;
            var stiffnessMatrix1D = DenseMatrix.OfArray(new[,] {{k, -k}, {-k, k}});

            var rotationMatrix = GetRotationMatrix(dir);
            return rotationMatrix * stiffnessMatrix1D * rotationMatrix.Transpose();
        }

        /// <summary>
        /// Mass matrix taking into account the shape functions.
        /// </summary>
        public Matrix<float> GetConsistentMassMatrix(IList<Node> nodes)
        {
            Vector2 dir;
            float length;
            GetDirAndLength(nodes, out dir, out length);

            var m = Density * CrossSectionalArea * length / 6.0f;
            var massMatrix1D =  DenseMatrix.OfArray(new[,] {{Density * 2, Density}, {Density, Density * 2}});
            
            var rotationMatrix = GetRotationMatrix(dir);
            return rotationMatrix * massMatrix1D * rotationMatrix.Transpose();
        }

        /// <summary>
        /// Daming matrix taking into account the shape function.
        /// </summary>
        /// <returns></returns>
        public Matrix<float> GetConsistentDampingMatrix(IList<Node> nodes)
        {
            Vector2 dir;
            float length;
            GetDirAndLength(nodes, out dir, out length);
            
            var dampingMatrix1d =  DenseMatrix.OfArray(new[,] {{DampingCoefficient * 2 / 6.0f, DampingCoefficient / 6.0f}, 
                                                               {DampingCoefficient / 6.0f, DampingCoefficient * 2 / 6.0f}});
            var rotationMatrix = GetRotationMatrix(dir);
            return rotationMatrix * dampingMatrix1d * rotationMatrix.Transpose();
        }
    }

    [Serializable]
    public struct Force
    {
        public Vector2 Vector;
        public int NodeIndex;
    }
    
    public List<Node> Nodes = new List<Node>(
        new[]
        {
            new Node {Position = Vector2.zero, Fixed = true},
            new Node {Position = Vector2.right, Fixed = false}
        });

    public List<Element> Elements = new List<Element>(new[] { new Element(leftNodeIdx: 0, rightNodeIdx: 1) } );

    public Force[] Forces = new Force[0];

    public bool Dynamic = true;
    [Range(0.0f, 0.5f)]
    public float NewMarkBeta = 0.25f;
    [Range(0.0f, 1.0f)]
    public float NewMarkGamma = 0.5f;
    
    private Vector<float> activeDisplacement = null;
    private Vector<float> speeds = null;
    private Vector<float> accelerations = null;

    void Start()
    {
        activeDisplacement = DenseVector.Create(Nodes.Count * 2, 0.0f);
        speeds = DenseVector.Create(Nodes.Count * 2, 0.0f);
        accelerations = DenseVector.Create(Nodes.Count * 2, 0.0f);
    }
    
    void Update()
    {
        var stiffness = ComputeGlobalStiffnessMatrix();
        var force = ComputeForceVector();

        if (Dynamic)
        {
            //float dT = Time.deltaTime;
            //float dTSquared = Time.deltaTime * dTSquared;
            // todo.
        }
        else
        {
            ApplyConstraints(stiffness);
            stiffness.Solve(force, activeDisplacement);           
        }
    }

    private Matrix<float> ComputeGlobalStiffnessMatrix()
    {
        return ComputeGlobalMatrix((e, n) => e.GetStiffnessMatrix(n));
    }
    private Matrix<float> ComputeGlobalMassMatrix()
    {
        return ComputeGlobalMatrix((e, n) => e.GetConsistentMassMatrix(n));
    }
    private Matrix<float> ComputeGlobalDampingMatrix()
    {
        return ComputeGlobalMatrix((e, n) => e.GetConsistentDampingMatrix(n));
    }
    
    private Matrix<float> ComputeGlobalMatrix(Func<Element, IList<Node>, Matrix<float>> getElementMatrixFunc)
    {
        Matrix<float> stiffnessMatrix = DenseMatrix.Create(Nodes.Count * 2, Nodes.Count * 2, 0.0f);

        foreach (var e in Elements)
        {
            var stiffnessElementMatrix = getElementMatrixFunc(e, Nodes);

            for (int r = 0; r < 4; ++r)
            {
                int globalR = (r < 2 ? e.LeftNodeIdx : e.RightNodeIdx) * 2 + r % 2;
                for (int c = 0; c < 4; ++c)
                {
                    int globalC = (c < 2 ? e.LeftNodeIdx : e.RightNodeIdx) * 2 + c % 2;
                    
                    stiffnessMatrix[globalR, globalC] += stiffnessElementMatrix[r, c];
                }
            }
        }
        
        return stiffnessMatrix;
    }

    private void ApplyConstraints(Matrix<float> globalMatrix)
    {
        for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
        {
            if (Nodes[nodeIdx].Fixed)
            {
                globalMatrix.ClearRows(nodeIdx*2, nodeIdx*2 + 1);
                globalMatrix.ClearColumns(nodeIdx*2, nodeIdx*2 + 1);
                globalMatrix[nodeIdx*2, nodeIdx*2] = 1.0f;
                globalMatrix[nodeIdx*2+1, nodeIdx*2+1] = 1.0f;
            }
        }
    }
    
    private Vector<float> ComputeForceVector()
    {
        Vector<float> forceVector = DenseVector.Create(Nodes.Count * 2, 0.0f);

        foreach (var f in Forces)
        {
            forceVector[f.NodeIndex * 2] += f.Vector.x;
            forceVector[f.NodeIndex * 2 + 1] += f.Vector.y;
        }

        return forceVector;
    }

    public Vector2 GetNodeWorldPosition(int nodeIdx)
    {
        if (nodeIdx < 0 || nodeIdx >= Nodes.Count)
            return Vector2.zero;
        
        var pos = Nodes[nodeIdx].Position + transform.position.To2D();
        if (activeDisplacement != null)
            pos += new Vector2(activeDisplacement[nodeIdx * 2], activeDisplacement[nodeIdx * 2 + 1]);
        return pos;
    }

    private void OnDrawGizmos()
    {
        // Elements
        Gizmos.color = Color.black;
        foreach (var e in Elements)
        {
            Gizmos.DrawLine(GetNodeWorldPosition(e.RightNodeIdx), GetNodeWorldPosition(e.LeftNodeIdx));
        }
    }
    
    void OnValidate()
    {
        EnsureValidNodes();
        EnsureValidElements();
    }

    private void EnsureValidNodes()
    {
        if (Nodes == null)
            return;

        while(Nodes.Count < 2)
            Nodes.Add(new Node());
    }

    private void EnsureValidElements()
    {
        if (Elements == null)
            return;

        for (int i = 0; i < Elements.Count; ++i)
        {
            if (Elements[i].LeftNodeIdx >= Nodes.Count)
                Elements[i].LeftNodeIdx = Nodes.Count - 1;

            if (Elements[i].RightNodeIdx >= Nodes.Count)
                Elements[i].RightNodeIdx = Nodes.Count - 1;
        }
    }
}