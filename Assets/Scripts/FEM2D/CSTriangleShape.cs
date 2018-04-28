using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEditor;
using UnityEngine;

// A triangle shape consisting of Constant Strain Triangles (CST)
// 
// As the name implies the strain is the same across the entire triangle. This leads to discontinuities in strain from one triangle to the next.
public class CSTriangleShape : FEMShape2D<CSTriangleElement>
{
    protected override int NodesPerElement => 3;

    protected override void Start()
    {
        base.Start();
        foreach (var e in Elements)
            e.FixPointOrder(Nodes);
    }

    protected override void OnDrawGizmos()
    {
        // Elements
        Gizmos.color = Color.black;
        foreach (var e in Elements)
        {
            Gizmos.DrawLine(GetNodeWorldPosition(e.NodeIdxA), GetNodeWorldPosition(e.NodeIdxB));
            Gizmos.DrawLine(GetNodeWorldPosition(e.NodeIdxB), GetNodeWorldPosition(e.NodeIdxC));
            Gizmos.DrawLine(GetNodeWorldPosition(e.NodeIdxC), GetNodeWorldPosition(e.NodeIdxA));
        }

        base.OnDrawGizmos();
    }
    
    protected override void EnsureValidElements()
    {
        if (Elements == null)
            return;

        foreach (var e in Elements)
        {
            if (e.NodeIdxA >= Nodes.Count)
                e.NodeIdxA = Nodes.Count - 1;

            if (e.NodeIdxB >= Nodes.Count)
                e.NodeIdxB = Nodes.Count - 1;
            
            if (e.NodeIdxB >= Nodes.Count)
                e.NodeIdxB = Nodes.Count - 1;
        }
    }

    protected override void ApplyGravityToForceVector(Vector<float> forceVector, Vector2 scaledGravity)
    {
        foreach (var element in Elements)
        {
            // Load is equally distributed on all nodes.
            float elementWeight = element.GetTotalMass(Nodes) / 3.0f;
            scaledGravity *= elementWeight;

            forceVector[element.NodeIdxA*2] += scaledGravity.x;
            forceVector[element.NodeIdxA*2+1] += scaledGravity.y;
            forceVector[element.NodeIdxB*2] += scaledGravity.x;
            forceVector[element.NodeIdxB*2+1] += scaledGravity.y;
            forceVector[element.NodeIdxC*2] += scaledGravity.x;
            forceVector[element.NodeIdxC*2+1] += scaledGravity.y;
        }
    }
}