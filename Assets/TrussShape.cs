﻿using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEditor;
using UnityEngine;

using Node = FEMShape2D.Node;

// A 2D structure whose line elements can experience only axial stress
//
// Also known as "Bernoulli-Euler Plane Beam"
public class TrussShape : MonoBehaviour
{
    [Serializable]
    public class Element : FEMShape2D.Element
    {
        [Tooltip("Area in square meter.")]
        public float CrossSectionalArea = 0.01f;    // For a rod: Thickness * Thickness * (float) Math.PI;
        
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
            var massMatrix1D =  DenseMatrix.OfArray(new[,] {{m * 2, m}, {m, m * 2}});
            
            var rotationMatrix = GetRotationMatrix(dir);
            return rotationMatrix * massMatrix1D * rotationMatrix.Transpose();
        }

        /// <summary>
        /// Lumped mass vector / diagonal matrix
        /// </summary>
        /// <remarks>
        /// We're using "Direct Mass Lumping" here which ignores any cross coupling of nodes amonst each other.
        /// This preserves translational kinetic energy but does not preserve angular momentum.
        /// 
        /// According to http://kis.tu.kielce.pl/mo/COLORADO_FEM/colorado/IFEM.Ch31.pdf a "real" lumped matrix will be singular and thus we would no longer be able to longe solve our equations.
        /// Therefore, a values between 0 and 1/50 * l*l is used in the rotational diagonal fields.
        /// </remarks>
        /// <returns></returns>
        public Vector<float> GetLumpedMassVector(IList<Node> nodes)
        {
            Vector2 dir;
            float length;
            GetDirAndLength(nodes, out dir, out length);

            var m = Density * CrossSectionalArea * length;
            var dummy = 1.0f / 50.0f * length * length;
            return DenseVector.OfArray(new[] {0.5f, dummy, 0.5f, dummy}) * m;
        }
        
        /// <summary>
        /// Damping matrix taking into account the shape function.
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
        /// <summary>
        /// Lumped mass vector / diagonal matrix
        /// </summary>
        /// <remarks>See GetLumpedMassvector</remarks>
        public Vector<float> GetLumpedDampingVector(IList<Node> nodes)
        {
            return DenseVector.OfArray(new[] {DampingCoefficient / 4, DampingCoefficient / 4, DampingCoefficient / 4, DampingCoefficient / 4});
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

    public enum Animation
    {
        DynamicImplicit,
        DynamicExplicitConsistent,
        DynamicExplicitLumped,
        Static,
    }
    
    public Animation AnimationMode = Animation.Static;
    [Range(0.0f, 0.5f)]
    public float NewMarkBeta = 0.25f;
    [Range(0.0f, 1.0f)]
    public float NewMarkGamma = 0.5f;
    
    private Vector<float> nodeDisplacement = null;
    private Vector<float> nodeDisplacementOld = null;
    private Vector<float> nodeSpeed = null;
    private Vector<float> nodeAcceleration = null;

    void Start()
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

    void UpdateStatic()
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
    
    void UpdateDynamicExplicit_Consistent(float dT)
    {
        // https://www.youtube.com/watch?v=YqynfK8qwFI&t=202s - Schuster Engineering, FEA 22: Transient Explicit
        var stiffness = ComputeGlobalStiffnessMatrix();
        var force = ComputeForceVector();
        var damping = ComputeGlobalConsistentDampingMatrix();
        var mass = ComputeGlobalConsistentMassMatrix();

        const int numIterations = 10;
        dT /= numIterations;
        for (int i = 0; i < numIterations; ++i)
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
    
    void UpdateDynamicExplicit_Lumped(float dT)
    {
        // https://www.youtube.com/watch?v=YqynfK8qwFI&t=202s - Schuster Engineering, FEA 22: Transient Explicit
        var stiffness = ComputeGlobalStiffnessMatrix();
        var force = ComputeForceVector();
        var damping = ComputeGlobalLumpedDampingVector();
        var mass = ComputeGlobalLumpedMassVector();

        const int numIterations = 20;
        dT /= numIterations;
        for (int i = 0; i < numIterations; ++i)
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
    
    void UpdateDynamicImplicit(float dT)
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
        force += ( 1.0f / (NewMarkBeta * dT) * mass + (NewMarkGamma / NewMarkBeta - 1.0f) * damping) * nodeSpeed;
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
    
    private Matrix<float> ComputeGlobalMatrix(Func<Element, IList<Node>, Matrix<float>> getElementMatrixFunc)
    {
        Matrix<float> matrix = DenseMatrix.Create(Nodes.Count * 2, Nodes.Count * 2, 0.0f);

        foreach (var e in Elements)
        {
            var elementMatrix = getElementMatrixFunc(e, Nodes);

            for (int r = 0; r < 4; ++r)
            {
                int globalR = (r < 2 ? e.LeftNodeIdx : e.RightNodeIdx) * 2 + r % 2;
                for (int c = 0; c < 4; ++c)
                {
                    int globalC = (c < 2 ? e.LeftNodeIdx : e.RightNodeIdx) * 2 + c % 2;
                    
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
    
    private Vector<float> ComputeGlobalVector(Func<Element, IList<Node>, Vector<float>> getElementVectorFunc)
    {
        Vector<float> vector = DenseVector.Create(Nodes.Count * 2, 0.0f);

        foreach (var e in Elements)
        {
            var elementVector = getElementVectorFunc(e, Nodes);
            for (int r = 0; r < 4; ++r)
            {
                int globalR = (r < 2 ? e.LeftNodeIdx : e.RightNodeIdx) * 2 + r % 2;
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
                matrix.ClearRows(nodeIdx*2, nodeIdx*2 + 1);
                matrix.ClearColumns(nodeIdx*2, nodeIdx*2 + 1);
                matrix[nodeIdx*2, nodeIdx*2] = 1.0f;
                matrix[nodeIdx*2 + 1, nodeIdx*2 + 1] = 1.0f;
            }
        }
    }
    
    private void ApplyConstraints(Vector<float> vector)
    {
        for (int nodeIdx = 0; nodeIdx < Nodes.Count; ++nodeIdx)
        {
            if (Nodes[nodeIdx].Fixed)
            {
                vector[nodeIdx*2] = 0.0f;
                vector[nodeIdx*2 + 1] = 0.0f;
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
        if (nodeDisplacement != null)
            pos += new Vector2(nodeDisplacement[nodeIdx * 2], nodeDisplacement[nodeIdx * 2 + 1]);
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