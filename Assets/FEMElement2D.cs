using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

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
public struct Force
{
    public Vector2 Vector;
    public int NodeIndex;
}

[Serializable]
public abstract class FEMElement2D
{
    [Tooltip("Thickness of the element in meters")]
    public float Thickness = 0.001f;
        
    [Tooltip("Describes stiffness of material. Rubber has 0.01-0.1, steel has 209")]
    public float YoungModulusGPa = 1.0f;

    public float YoungModulus => YoungModulusGPa * 1000000000;
        
    [Tooltip("kg per cubic meter. 1000 is water.")]
    public float Density = 1000.0f;

    public float DampingCoefficient = 50.0f;

        
        
    public abstract Matrix<float> GetStiffnessMatrix(IList<Node> nodes);

    /// <summary>
    /// Mass matrix taking into account the shape functions.
    /// </summary>
    public abstract Matrix<float> GetConsistentMassMatrix(IList<Node> nodes);

    /// <summary>
    /// Lumped mass vector / diagonal matrix
    /// </summary>
    public abstract Vector<float> GetLumpedMassVector(IList<Node> nodes);

    /// <summary>
    /// Damping matrix taking into account the shape function.
    /// </summary>
    /// <returns></returns>
    public abstract Matrix<float> GetConsistentDampingMatrix(IList<Node> nodes);

    /// <summary>
    /// Lumped mass vector / diagonal matrix
    /// </summary>
    public abstract Vector<float> GetLumpedDampingVector(IList<Node> nodes);


    public abstract int LocalComponentIndexToGlobal(int local);
}