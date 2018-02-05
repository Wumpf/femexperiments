using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEngine;

[Serializable]
public class TrussElement : FEMElement2D
{
    [Tooltip("Area in square meter.")]
    public float CrossSectionalArea = 0.01f;    // For a rod: Thickness * Thickness * (float) Math.PI;
    
    public int LeftNodeIdx;
    public int RightNodeIdx;

    public TrussElement(int leftNodeIdx, int rightNodeIdx)
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
    
    public override Matrix<float> GetStiffnessMatrix(IList<Node> nodes)
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

    public override Matrix<float> GetConsistentMassMatrix(IList<Node> nodes)
    {
        Vector2 dir;
        float length;
        GetDirAndLength(nodes, out dir, out length);

        var m = Density * CrossSectionalArea * length / 6.0f;
        var massMatrix1D =  DenseMatrix.OfArray(new[,] {{m * 2, m}, {m, m * 2}});
        
        var rotationMatrix = GetRotationMatrix(dir);
        return rotationMatrix * massMatrix1D * rotationMatrix.Transpose();
    }

    /// <remarks>
    /// We're using "Direct Mass Lumping" here which ignores any cross coupling of nodes amonst each other.
    /// This preserves translational kinetic energy but does not preserve angular momentum.
    /// 
    /// According to http://kis.tu.kielce.pl/mo/COLORADO_FEM/colorado/IFEM.Ch31.pdf a "real" lumped matrix will be singular and thus we would no longer be able to longe solve our equations.
    /// Therefore, a values between 0 and 1/50 * l*l is used in the rotational diagonal fields.
    /// </remarks>
    /// <returns></returns>
    public override Vector<float> GetLumpedMassVector(IList<Node> nodes)
    {
        Vector2 dir;
        float length;
        GetDirAndLength(nodes, out dir, out length);

        var m = Density * CrossSectionalArea * length;
        var dummy = 1.0f / 50.0f * length * length;
        return DenseVector.OfArray(new[] {0.5f, dummy, 0.5f, dummy}) * m;
    }
    
    public override  Matrix<float> GetConsistentDampingMatrix(IList<Node> nodes)
    {
        Vector2 dir;
        float length;
        GetDirAndLength(nodes, out dir, out length);
        
        var dampingMatrix1d =  DenseMatrix.OfArray(new[,] {{DampingCoefficient * 2 / 6.0f, DampingCoefficient / 6.0f}, 
                                                           {DampingCoefficient / 6.0f, DampingCoefficient * 2 / 6.0f}});
        var rotationMatrix = GetRotationMatrix(dir);
        return rotationMatrix * dampingMatrix1d * rotationMatrix.Transpose();
    }

    public override Vector<float> GetLumpedDampingVector(IList<Node> nodes)
    {
        return DenseVector.OfArray(new[] {DampingCoefficient / 4, DampingCoefficient / 4, DampingCoefficient / 4, DampingCoefficient / 4});
    }

    public override int LocalComponentIndexToGlobal(int local)
    {
        return (local < 2 ? LeftNodeIdx : RightNodeIdx) * 2 + local % 2;
    }
}