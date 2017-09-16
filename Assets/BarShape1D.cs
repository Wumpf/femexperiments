using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEngine.Playables;

public class BarShape1D : MonoBehaviour
{
    [System.Serializable]
    public struct Node
    {
        public float Position;
        public float PointLoad;
        public bool Fixed;
    }

    [System.Serializable]
    public class Element
    {
		public float Thickness;
		public float YoungModulus;

		public float DistributedLoad;

        public int LeftNodeIdx;
        public int RightNodeIdx;

		public float CrossSectionalArea
		{
			// Assume we are dealing with cylinders.
			get { return Thickness * Thickness * (float)Math.PI; }
		}

        private float GetLength(Node[] nodes)
        {
            return Math.Abs(nodes[RightNodeIdx].Position - nodes[LeftNodeIdx].Position);
        }

		public Matrix<float> GetStiffnessMatrix(Node[] nodes)
		{
			float k = CrossSectionalArea * YoungModulus / GetLength(nodes);

            return DenseMatrix.OfArray(new float[,] { { k, -k }, { -k, k } });
		}

        public Vector<float> GetDistributedLoadVector(Node[] nodes)
		{
            float f = 0.5f * DistributedLoad * GetLength(nodes);
            return DenseVector.OfArray(new float[] { f, f });
		}
    }

    public Node[] Nodes;
    public Element[] Elements;
    private Vector<float> activeDisplacement;


    // Use this for initialization
    void Start()
    {
    }

    void FixedUpdate()
    {
        var stiffness = ComputeGlobalStiffnessMatrix();
        var force = ComputeForceVector();
        activeDisplacement = stiffness.Solve(force);
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

        if (Nodes.Length == 1)
            Nodes = new Node[] { Nodes[0], new Node() };
        else if (Nodes.Length == 0)
            Nodes = new Node[2];
	}

    private void EnsureValidElements()
    {
        if (Elements == null)
            return;
        
        for (int i = 0; i < Elements.Length; ++i)
        {
            if (Elements[i].LeftNodeIdx >= Nodes.Length)
                Elements[i].LeftNodeIdx = Nodes.Length - 1;

			if (Elements[i].RightNodeIdx >= Nodes.Length)
				Elements[i].RightNodeIdx = Nodes.Length - 1;
        }
    }

    private Matrix<float> ComputeGlobalStiffnessMatrix()
    {
        Matrix<float> stiffnessMatrix = DenseMatrix.Create(Nodes.Length, Nodes.Length, 0.0f); //SparseMatrix.Create(Nodes.Length, Nodes.Length, 0.0f);

        foreach (var e in Elements)
        {
            var stiffnessElementMatrix = e.GetStiffnessMatrix(Nodes);
            stiffnessMatrix[e.LeftNodeIdx, e.LeftNodeIdx] += stiffnessElementMatrix[0, 0];
            stiffnessMatrix[e.RightNodeIdx, e.LeftNodeIdx] += stiffnessElementMatrix[1, 0];
            stiffnessMatrix[e.LeftNodeIdx, e.RightNodeIdx] += stiffnessElementMatrix[0, 1];
            stiffnessMatrix[e.RightNodeIdx, e.RightNodeIdx] += stiffnessElementMatrix[1, 1];
        }

        // Constrained nodes.
        for (int nodeIdx = 0; nodeIdx < Nodes.Length; ++nodeIdx)
        {
            if (Nodes[nodeIdx].Fixed)
            {
                stiffnessMatrix.ClearRow(nodeIdx);
                stiffnessMatrix[nodeIdx, nodeIdx] = 1.0f;
            }
        }

        return stiffnessMatrix;
    }

    private Vector<float> ComputeForceVector()
    {
        Vector<float> forceVector = DenseVector.Create(Nodes.Length, 0.0f);
        
        foreach (var e in Elements)
        {
            var distributedLoadVector = e.GetDistributedLoadVector(Nodes);
            forceVector[e.LeftNodeIdx] += distributedLoadVector[0];
            forceVector[e.RightNodeIdx] += distributedLoadVector[1];
        }

        for (int nodeIdx = 0; nodeIdx<Nodes.Length; ++nodeIdx)
        {
            if (Nodes[nodeIdx].Fixed)
                forceVector[nodeIdx] = 0;
            else
                forceVector[nodeIdx] += Nodes[nodeIdx].PointLoad;
        }

        return forceVector;
    }

    void OnDrawGizmos()
    {
		if (Elements == null)
			return;

		foreach(var e in Elements)
		{
		    float posA = Nodes[e.RightNodeIdx].Position;
		    float posB = Nodes[e.LeftNodeIdx].Position;

		    if (activeDisplacement != null)
		    {
		        posA += activeDisplacement[e.RightNodeIdx];
		        posB += activeDisplacement[e.LeftNodeIdx];
		    }
		    
            Gizmos.DrawCube(new Vector3((posA + posB) * 0.5f, 0.0f) + transform.position, 
                            new Vector3((posA - posB), e.Thickness));
		    
        }
    }
}
