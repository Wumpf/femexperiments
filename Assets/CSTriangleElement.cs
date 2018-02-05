using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEditor;
using UnityEngine;

[Serializable]
public class CSTriangleElement : FEMElement2D
{
    public int NodeIdxA;
    public int NodeIdxB;
    public int NodeIdxC;

    [Tooltip("Relationship between transverse strain and axial strain.\n" +
             "If a material is stretched/compressed in one direction, it gets thinner/thicker in the other two. Negative means that it expands in the other direction." +
             "A high ratio means more change in the other axis. Rubber has close to 0.5, cast steel has 0.265")]
    public float PoissonRatio = 0.3f;

    public CSTriangleElement(int nodeIdxA, int nodeIdxB, int nodeIdxC)
    {
        NodeIdxA = nodeIdxA;
        NodeIdxB = nodeIdxB;
        NodeIdxC = nodeIdxC;
    }

    public void FixPointOrder(IList<Node> nodes)
    {
        if (GetArea(nodes) < 0.0f)
        {
            var tmp = NodeIdxA;
            NodeIdxA = NodeIdxB;
            NodeIdxB = tmp;
        }
    }

    private float GetArea(IList<Node> nodes)
    {
        var pos1 = nodes[NodeIdxA].Position;
        var pos2 = nodes[NodeIdxB].Position;
        var pos3 = nodes[NodeIdxC].Position;

        var p13 = pos1 - pos3;
        var p23 = pos2 - pos3;

        return (p13.x * p23.y - p23.x * p13.y) * 0.5f;
    }

    private float GetTotalMass(IList<Node> nodes)
    {
        return Density * GetArea(nodes) * Thickness;
    }

    // Also called "Material's Matrix"
    private Matrix<float> GetStressStrainRelationMatrix()
    {
        return DenseMatrix.OfArray(new float[,]
        {
            {1.0f, PoissonRatio, 0.0f}, {PoissonRatio, 1.0f, 0.0f}, {0.0f, 0.0f, 0.5f * (1.0f - PoissonRatio)}
        }) * YoungModulus / (1.0f - PoissonRatio * PoissonRatio);
    }

    public override Matrix<float> GetStiffnessMatrix(IList<Node> nodes)
    {
        var pos1 = nodes[NodeIdxA].Position;
        var pos2 = nodes[NodeIdxB].Position;
        var pos3 = nodes[NodeIdxC].Position;

        var p13 = pos1 - pos3; // TODO: simplify?
        var p12 = pos1 - pos2;
        var p23 = pos2 - pos3;
        var p21 = pos2 - pos1;
        var p32 = pos3 - pos2;
        var p31 = pos3 - pos1;

        float area2 = p13.x * p23.y - p23.x * p13.y; // determinat of Jacobian = area times 2

        // Not exactly strain displacement matrix - missing division by 2A
        var strainDisplacementMatrix = DenseMatrix.OfArray(new float[,]
        {
            {p23.y, 0.0f, p31.y, 0.0f, p12.y, 0.0f}, {0.0f, p32.x, 0.0f, p13.x, 0.0f, p21.x}, {p32.x, p23.y, p13.x, p31.y, p21.x, p12.y}
        });

        var materialMatrix = GetStressStrainRelationMatrix();

        return (Thickness / (2.0f * area2)) * strainDisplacementMatrix.TransposeThisAndMultiply(materialMatrix * strainDisplacementMatrix);
    }

    public override Matrix<float> GetConsistentMassMatrix(IList<Node> nodes)
    {
        return GetTotalMass(nodes) / 12.0f * DenseMatrix.OfArray(new float[,]
        {
            {2.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 2.0f, 0.0f, 1.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 2.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 2.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f, 0.0f, 2.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 2.0f}
        });
    }

    /// <remarks>
    /// Mass is equally distributed to the three nodes
    /// </remarks>
    public override Vector<float> GetLumpedMassVector(IList<Node> nodes)
    {
        return DenseVector.Create(6, GetTotalMass(nodes) / 3.0f);
    }

    public override Matrix<float> GetConsistentDampingMatrix(IList<Node> nodes)
    {
        return DampingCoefficient / 12.0f * DenseMatrix.OfArray(new float[,]
        {
            {2.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 2.0f, 0.0f, 1.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 2.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 2.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f, 0.0f, 2.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 2.0f}
        });
    }

    /// <remarks>See GetLumpedMassvector</remarks>
    public override Vector<float> GetLumpedDampingVector(IList<Node> nodes)
    {
        return DenseVector.Create(6, DampingCoefficient / 3.0f);
    }

    public override int LocalComponentIndexToGlobal(int local)
    {
        int nodeIdx;
        switch (local)
        {
            case 0:
            case 1:
                nodeIdx = NodeIdxA;
                break;

            case 2:
            case 3:
                nodeIdx = NodeIdxB;
                break;

            case 4:
            case 5:
                nodeIdx = NodeIdxC;
                break;

            default:
                throw new ArgumentException();
        }

        return nodeIdx * 2 + local % 2;
    }
}
