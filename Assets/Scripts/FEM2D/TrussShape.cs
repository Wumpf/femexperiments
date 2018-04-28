using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEditor;
using UnityEngine;

// A 2D structure whose line elements can experience only axial stress
//
// Also known as "Bernoulli-Euler Plane Beam"
public class TrussShape : FEMShape2D<TrussElement>
{
    protected override int NodesPerElement => 2;

    protected override void OnDrawGizmos()
    {
        // Elements
        Gizmos.color = Color.black;
        foreach (var e in Elements)
        {
            Gizmos.DrawLine(GetNodeWorldPosition(e.RightNodeIdx), GetNodeWorldPosition(e.LeftNodeIdx));
        }
        
        base.OnDrawGizmos();
    }
    
    protected override void EnsureValidElements()
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

    protected override void ApplyGravityToForceVector(Vector<float> forceVector, Vector2 scaledGravity)
    {
        foreach (var element in Elements)
        {
            // f = g * m = g * (d * l * a)
            float elementWeight = element.Density * element.CrossSectionalArea * (Nodes[element.LeftNodeIdx].Position - Nodes[element.RightNodeIdx].Position).magnitude * 0.5f;
            scaledGravity *= elementWeight;

            forceVector[element.LeftNodeIdx*2] += scaledGravity.x;
            forceVector[element.LeftNodeIdx*2+1] += scaledGravity.y;
            forceVector[element.RightNodeIdx*2] += scaledGravity.x;
            forceVector[element.RightNodeIdx*2+1] += scaledGravity.y;
        }
    }
}