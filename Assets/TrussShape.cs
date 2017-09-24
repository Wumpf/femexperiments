using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEditor;
using UnityEngine;
using UnityEngine.VR;
using UnityEngine.VR.WSA;

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

        public int LeftNodeIdx;
        public int RightNodeIdx;

        public Element(int leftNodeIdx, int rightNodeIdx)
        {
            LeftNodeIdx = leftNodeIdx;
            RightNodeIdx = rightNodeIdx;
        }
        
        public Matrix<float> GetStiffnessMatrix(IList<Node> nodes)
        {
            Vector2 dir = nodes[RightNodeIdx].Position - nodes[LeftNodeIdx].Position;
            float length = dir.magnitude;
            float k = CrossSectionalArea * YoungModulus / length;
            var stiffnessMatrix1D = DenseMatrix.OfArray(new[,] {{k, -k}, {-k, k}});

            float cosAngle = dir.x / length; // dot product with x axis divided by length.
            float sinAngle = (float)Math.Sqrt(1.0f - cosAngle * cosAngle); // http://www.wolframalpha.com/input/?i=sin(cos%CB%86-1(x))
            var rotationMatrix = DenseMatrix.OfArray(new[,] {{cosAngle, 0.0f}, {sinAngle, 0.0f}, {0.0f, cosAngle}, {0.0f, sinAngle}});

            return rotationMatrix * stiffnessMatrix1D * rotationMatrix.Transpose();
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
    
    private Vector<float> activeDisplacement = null;
//    private Vector<float> elementStrains;
    
    // Update is called once per frame
    void Update()
    {
        var stiffness = ComputeGlobalStiffnessMatrix();
        var force = ComputeForceVector();
        activeDisplacement = stiffness.Solve(force);
//
//        for (int i = 0; i < Elements.Length; ++i)
//        {
//            var e = Elements[i];
//            float elementLengthInv = 1.0f / e.GetLength(Nodes);
//            elementStrains[i] = e.YoungModulus *
//                                (-elementLengthInv * activeDisplacement[e.LeftNodeIdx] +
//                                 elementLengthInv * activeDisplacement[e.RightNodeIdx]);
//        }
    }
    
    private Matrix<float> ComputeGlobalStiffnessMatrix()
    {
        Matrix<float> stiffnessMatrix = DenseMatrix.Create(Nodes.Count * 2, Nodes.Count * 2, 0.0f);

        foreach (var e in Elements)
        {
            var stiffnessElementMatrix = e.GetStiffnessMatrix(Nodes);

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

        // Constrained nodes.
        for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
        {
            if (Nodes[nodeIdx].Fixed)
            {
                stiffnessMatrix.ClearRows(nodeIdx*2, nodeIdx*2 + 1);
                stiffnessMatrix.ClearColumns(nodeIdx*2, nodeIdx*2 + 1);
                stiffnessMatrix[nodeIdx*2, nodeIdx*2] = 1.0f;
                stiffnessMatrix[nodeIdx*2+1, nodeIdx*2+1] = 1.0f;
            }
        }

        return stiffnessMatrix;
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
        //elementStrains = DenseVector.Create(Elements.Length, 0.0f);
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