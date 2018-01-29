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
public class TriangleShape : MonoBehaviour
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
        public int NodeIdxA;
        public int NodeIdxB;
        public int NodeIdxC;

        public Element(int nodeIdxA, int nodeIdxB, int nodeIdxC)
        {
            NodeIdxA = nodeIdxA;
            NodeIdxB = nodeIdxB;
            NodeIdxC = nodeIdxC;
        }
        
        
        
        public Matrix<float> GetStiffnessMatrix(IList<Node> nodes)
        {
            // TODO
            return DenseMatrix.Create(1,1,0);
        }

        /// <summary>
        /// Mass matrix taking into account the shape functions.
        /// </summary>
        public Matrix<float> GetConsistentMassMatrix(IList<Node> nodes)
        {
            // TODO
            return DenseMatrix.Create(1,1,0);
        }

        /// <summary>
        /// Lumped mass vector / diagonal matrix
        /// </summary>
        /// <remarks>
        /// Since we want this to be in global space, we can not really model the mass distribution accurately.
        /// -> All representations I could find, defined the lumped mass (diagonal-)matrix in traverse+rotation space (as opposed to x+y).
        /// There, thew two entries for transverse "action" would have m/2 while rotational influence would be zero.
        /// Since everything else here is in global space, I don't see how to apply this. Instead I came up with this hack which I think might suffer from problems depending on the rotation of the element.
        /// </remarks>
        /// <returns></returns>
        public Vector<float> GetLumpedMassVector(IList<Node> nodes)
        {
            // TODO
            return DenseVector.Create(1,0);
        }
        
        /// <summary>
        /// Damping matrix taking into account the shape function.
        /// </summary>
        /// <returns></returns>
        public Matrix<float> GetConsistentDampingMatrix(IList<Node> nodes)
        {
            // TODO
            return DenseMatrix.Create(1,1,0);
        }
        /// <summary>
        /// Lumped mass vector / diagonal matrix
        /// </summary>
        /// <remarks>See GetLumpedMassvector</remarks>
        public Vector<float> GetLumpedDampingVector(IList<Node> nodes)
        {
            // TODO
            return DenseVector.Create(1,0);
        }

        public int LocalComponentIndexToGlobal(int local)
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
                    nodeIdx = NodeIdxA;
                    break;
                    
                case 4:
                case 5:
                    nodeIdx = NodeIdxA;
                    break;
                
                default:
                    throw new ArgumentException();
            }

            return nodeIdx * 2 + local % 2;
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
            new Node {Position = Vector2.up, Fixed = true},
            new Node {Position = Vector2.right, Fixed = false}
        });

    public List<Element> Elements = new List<Element>(new[] { new Element(0, 1, 2) } );

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

            for (int r = 0; r < 6; ++r)
            {
                int globalR = e.LocalComponentIndexToGlobal(r);
                for (int c = 0; c < 6; ++c)
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
    
    private Vector<float> ComputeGlobalVector(Func<Element, IList<Node>, Vector<float>> getElementVectorFunc)
    {
        Vector<float> vector = DenseVector.Create(Nodes.Count * 2, 0.0f);

        foreach (var e in Elements)
        {
            var elementVector = getElementVectorFunc(e, Nodes);
            for (int r = 0; r < 4; ++r)
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
            Gizmos.DrawLine(GetNodeWorldPosition(e.NodeIdxA), GetNodeWorldPosition(e.NodeIdxB));
            Gizmos.DrawLine(GetNodeWorldPosition(e.NodeIdxB), GetNodeWorldPosition(e.NodeIdxC));
            Gizmos.DrawLine(GetNodeWorldPosition(e.NodeIdxC), GetNodeWorldPosition(e.NodeIdxA));
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
}