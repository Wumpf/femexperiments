using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEditor;
using UnityEngine;

public abstract class FEMShape2D<TElementType> : MonoBehaviour
    where TElementType : FEMElement2D
{
    public Force[] Forces = new Force[0];

    public float GravityScale = 1.0f;
    
    public enum Animation
    {
        DynamicImplicit,
        DynamicExplicitConsistent,
        DynamicExplicitLumped,
        Static
    }

    public Animation AnimationMode = Animation.Static;

    [Range(0.0f, 0.5f)]
    public float NewMarkBeta = 0.25f;
    [Range(0.0f, 1.0f)]
    public float NewMarkGamma = 0.5f;

    public List<Node> Nodes = new List<Node>();
    public List<TElementType> Elements = new List<TElementType>();

    protected abstract int NodesPerElement { get; }

    protected Vector<float> nodeDisplacement;
    private Vector<float> nodeDisplacementOld;
    private Vector<float> nodeSpeed;
    private Vector<float> nodeAcceleration;

    private void ResetState()
    {
        // Estimate d_-1 using euler time formula. F=ma => a = F/m
        var test = ComputeGlobalConsistentMassMatrix();
        ApplyConstraints(test);
        var startupAcceleration = test.Solve(ComputeForceVector());
        nodeDisplacementOld = startupAcceleration * (Time.fixedDeltaTime * Time.fixedDeltaTime * 0.5f);
        ApplyConstraints(nodeDisplacementOld);

        nodeDisplacement = DenseVector.Create(Nodes.Count * 2, 0.0f);
        nodeSpeed = DenseVector.Create(Nodes.Count * 2, 0.0f);
        nodeAcceleration = DenseVector.Create(Nodes.Count * 2, 0.0f);
    }

    protected virtual void Start()
    {
        ResetState();
    }

    private void UpdateStatic()
    {
        var stiffness = ComputeGlobalStiffnessMatrix();
        var force = ComputeForceVector();
        ApplyConstraints(stiffness);
        ApplyConstraints(force);
        stiffness.Solve(force, nodeDisplacement);
    }

    // For all dynamic systems keep this simply equation in mind:
    //
    // mass * accelleration + damping * speed + stiffness * displacement = force
    //
    // Note that: 
    // * acceleration = displacement''
    // * speed = displacement'

    private const int numIterationsPerUpdateForExplicitAnalysis = 30; 
    
    private void UpdateDynamicExplicit_Consistent(float dT)
    {
        // https://www.youtube.com/watch?v=YqynfK8qwFI&t=202s - Schuster Engineering, FEA 22: Transient Explicit
        var stiffness = ComputeGlobalStiffnessMatrix();
        var force = ComputeForceVector();
        var damping = ComputeGlobalConsistentDampingMatrix();
        var mass = ComputeGlobalConsistentMassMatrix();

        dT /= numIterationsPerUpdateForExplicitAnalysis;
        for (int i = 0; i < numIterationsPerUpdateForExplicitAnalysis; ++i)
        {
            float dTInvSq = 1.0f / (dT * dT);
            float dTInv2 = 1.0f / (dT * 2.0f);

            // "left hand"
            var leftHand = dTInvSq * mass + dTInv2 * damping; // not sure what to call a mass + damping thing

            // "right hand"
            var currentContrib = ((2.0f * dTInvSq) * mass - stiffness) * nodeDisplacement;
            var pastContrib = (dTInvSq * mass - dTInv2 * damping) * nodeDisplacementOld;
            var rightHand = force + currentContrib - pastContrib; // not sure what to call a ... whatever that is

            // solve!
            ApplyConstraints(rightHand);
            ApplyConstraints(leftHand);
            nodeDisplacementOld = nodeDisplacement;
            nodeDisplacement = leftHand.Solve(rightHand);
        }

        // computing displacement and speed directly from this requires some thought (it's in there!), but can of course always be done using central differences.
    }

    private void UpdateDynamicExplicit_Lumped(float dT)
    {
        // https://www.youtube.com/watch?v=YqynfK8qwFI&t=202s - Schuster Engineering, FEA 22: Transient Explicit
        var stiffness = ComputeGlobalStiffnessMatrix();
        var force = ComputeForceVector();
        var damping = ComputeGlobalLumpedDampingVector();
        var mass = ComputeGlobalLumpedMassVector();

        dT /= numIterationsPerUpdateForExplicitAnalysis;
        for (int i = 0; i < numIterationsPerUpdateForExplicitAnalysis; ++i)
        {
            float dTInvSq = 1.0f / (dT * dT);
            float dTInv2 = 1.0f / (dT * 2.0f);

            // "left hand"
            var leftHand = dTInvSq * mass + dTInv2 * damping; // not sure what to call a mass + damping thing

            // "right hand"
            var currentContrib = (DiagonalMatrix.OfDiagonal(mass.Count, mass.Count, (2.0f * dTInvSq) * mass) - stiffness) * nodeDisplacement;
            var pastContrib = (dTInvSq * mass - dTInv2 * damping).PointwiseMultiply(nodeDisplacementOld);
            var rightHand = force + currentContrib - pastContrib; // not sure what to call a ... whatever that is

            // solve!
            ApplyConstraints(rightHand);
            ApplyConstraints(leftHand);
            nodeDisplacementOld = nodeDisplacement;
            nodeDisplacement = rightHand.PointwiseDivide(leftHand);
            ApplyConstraints(nodeDisplacement);
        }

        // computing displacement and speed directly from this requires some thought (it's in there!), but can of course always be done using central differences.
    }

    private void UpdateDynamicImplicit(float dT)
    {
        // https://www.youtube.com/watch?v=BBzHdHgqxfE&t=714s - Schuster Engineering, FEA 21: Transient Implicit
        var stiffness = ComputeGlobalStiffnessMatrix();
        var force = ComputeForceVector();
        var damping = ComputeGlobalConsistentDampingMatrix();
        var mass = ComputeGlobalConsistentMassMatrix();

        // Not sure if this is the right name, but it certainly has a lot to do with inertia.
        var ineratiaMatrix = 1.0f / (NewMarkBeta * dT * dT) * mass +
                             NewMarkGamma / (NewMarkBeta * dT) * damping;

        // Effective stiffness matrix.
        stiffness += ineratiaMatrix;

        // Effective force.
        force += ineratiaMatrix * nodeDisplacement;
        force += (1.0f / (NewMarkBeta * dT) * mass + (NewMarkGamma / NewMarkBeta - 1.0f) * damping) * nodeSpeed;
        force += ((1.0f / (2.0f * NewMarkBeta) - 1.0f) * mass + (NewMarkGamma / (2.0f * NewMarkBeta) - 1.0f) * dT * damping) * nodeAcceleration;

        // Solve for new displacement
        ApplyConstraints(stiffness);
        ApplyConstraints(force);
        var nodeDisplacementNew = stiffness.Solve(force);
        var nodeDisplacementChange = nodeDisplacementNew - nodeDisplacement;

        // Update speed
        var nodeSpeedNew = (1.0f - NewMarkGamma / NewMarkBeta) * nodeSpeed +
                           dT * (1.0f - NewMarkGamma / (2.0f * NewMarkBeta)) * nodeAcceleration +
                           NewMarkGamma / (NewMarkBeta * dT) * nodeDisplacementChange;
        // Update accelleration
        nodeAcceleration = 1.0f / (NewMarkBeta * dT * dT) * (nodeDisplacementChange - dT * nodeSpeed) +
                           (1.0f - 1.0f / (2.0f * NewMarkBeta)) * nodeAcceleration;

        nodeDisplacement = nodeDisplacementNew;
        nodeSpeed = nodeSpeedNew;
    }

    void FixedUpdate()
    {
        if (Input.GetKeyDown(KeyCode.R))
            ResetState();
        
        switch (AnimationMode)
        {
            case Animation.DynamicExplicitLumped:
                UpdateDynamicExplicit_Lumped(Time.fixedDeltaTime);
                break;
            case Animation.DynamicExplicitConsistent:
                UpdateDynamicExplicit_Consistent(Time.fixedDeltaTime);
                break;
            case Animation.DynamicImplicit:
                UpdateDynamicImplicit(Time.fixedDeltaTime);
                break;
            case Animation.Static:
                UpdateStatic();
                break;
        }
    }

    private Matrix<float> ComputeGlobalStiffnessMatrix()
    {
        return ComputeGlobalMatrix((e, n) => e.GetStiffnessMatrix(n));
    }

    private Matrix<float> ComputeGlobalConsistentMassMatrix()
    {
        return ComputeGlobalMatrix((e, n) => e.GetConsistentMassMatrix(n));
    }

    private Matrix<float> ComputeGlobalConsistentDampingMatrix()
    {
        return ComputeGlobalMatrix((e, n) => e.GetConsistentDampingMatrix(n));
    }

    private Matrix<float> ComputeGlobalMatrix(Func<TElementType, IList<Node>, Matrix<float>> getElementMatrixFunc)
    {
        Matrix<float> matrix = DenseMatrix.Create(Nodes.Count * 2, Nodes.Count * 2, 0.0f);

        foreach (var e in Elements)
        {
            var elementMatrix = getElementMatrixFunc(e, Nodes);

            for (int r = 0; r < NodesPerElement * 2; ++r)
            {
                int globalR = e.LocalComponentIndexToGlobal(r);
                for (int c = 0; c < NodesPerElement * 2; ++c)
                {
                    int globalC = e.LocalComponentIndexToGlobal(c);
                    matrix[globalR, globalC] += elementMatrix[r, c];
                }
            }
        }

        return matrix;
    }

    private Vector<float> ComputeGlobalLumpedMassVector()
    {
        return ComputeGlobalVector((e, n) => e.GetLumpedMassVector(n));
    }

    private Vector<float> ComputeGlobalLumpedDampingVector()
    {
        return ComputeGlobalVector((e, n) => e.GetLumpedDampingVector(n));
    }

    private Vector<float> ComputeGlobalVector(Func<TElementType, IList<Node>, Vector<float>> getElementVectorFunc)
    {
        Vector<float> vector = DenseVector.Create(Nodes.Count * 2, 0.0f);

        foreach (var e in Elements)
        {
            var elementVector = getElementVectorFunc(e, Nodes);
            for (int r = 0; r < NodesPerElement * 2; ++r)
            {
                int globalR = e.LocalComponentIndexToGlobal(r);
                vector[globalR] += elementVector[r];
            }
        }

        return vector;
    }

    private void ApplyConstraints(Matrix<float> matrix)
    {
        for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
        {
            if (Nodes[nodeIdx].Fixed)
            {
                matrix.ClearRows(nodeIdx * 2, nodeIdx * 2 + 1);
                matrix.ClearColumns(nodeIdx * 2, nodeIdx * 2 + 1);
                matrix[nodeIdx * 2, nodeIdx * 2] = 1.0f;
                matrix[nodeIdx * 2 + 1, nodeIdx * 2 + 1] = 1.0f;
            }
        }
    }

    private void ApplyConstraints(Vector<float> vector)
    {
        for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
        {
            if (Nodes[nodeIdx].Fixed)
            {
                vector[nodeIdx * 2] = 0.0f;
                vector[nodeIdx * 2 + 1] = 0.0f;
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

        var scaledGravity = Physics2D.gravity * GravityScale;
        ApplyGravityToForceVector(forceVector, scaledGravity);

        return forceVector;
    }

    protected abstract void ApplyGravityToForceVector(Vector<float> forceVector, Vector2 scaledGravity);

    private void OnValidate()
    {
        EnsureValidNodes();
        EnsureValidElements();
    }

    private void EnsureValidNodes()
    {
        if (Nodes == null)
            return;

        while (Nodes.Count < NodesPerElement)
            Nodes.Add(new Node());
    }

    protected abstract void EnsureValidElements();
    
    public Vector3 GetNodeWorldPosition(int nodeIdx)
    {
        if (nodeIdx < 0 || nodeIdx >= Nodes.Count)
            return Vector2.zero;
        
        var pos = Nodes[nodeIdx].Position;
        if (nodeDisplacement != null)
            pos += new Vector2(nodeDisplacement[nodeIdx * 2], nodeDisplacement[nodeIdx * 2 + 1]);
        
        return transform.TransformPoint(pos.To3D());
    }
    
    protected virtual void OnDrawGizmos()
    {
        for(int i=0; i<Nodes.Count; ++i)
        {
            Gizmos.DrawSphere(GetNodeWorldPosition(i), 0.05f);
        }
    }
}