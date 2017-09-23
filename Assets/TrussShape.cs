using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
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
        public float Thickness { get; set; } = 1.0f;
        public float YoungModulus { get; set; } = 0.5f;

        public int LeftNodeIdx { get; private set; }
        public int RightNodeIdx { get; private set; }

        public Element(int leftNodeIdx, int rightNodeIdx)
        {
            LeftNodeIdx = leftNodeIdx;
            RightNodeIdx = rightNodeIdx;
        }
        
        // Assume we are dealing with cylinders.
        public float CrossSectionalArea => Thickness * Thickness * (float) Math.PI;

        public float GetLength(IList<Node> nodes)
        {
            return (nodes[RightNodeIdx].Position - nodes[LeftNodeIdx].Position).magnitude;
        }

        public Matrix<float> GetStiffnessMatrix(IList<Node> nodes)
        {
            float k = CrossSectionalArea * YoungModulus / GetLength(nodes);

            return DenseMatrix.OfArray(new[,] {{k, -k}, {-k, k}});
        }
    }
    
    public List<Node> Nodes = new List<Node>(
        new[]
        {
            new Node {Position = Vector2.zero, Fixed = true},
            new Node {Position = Vector2.right, Fixed = false}
        });

    public List<Element> Elements = new List<Element>(new[] { new Element(leftNodeIdx: 0, rightNodeIdx: 1) } );

    // Use this for initialization
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
    }

    private void OnDrawGizmos()
    {
        // Elements
        Gizmos.color = Color.black;
        foreach (var element in Elements)
        {
            Gizmos.DrawLine(Nodes[element.LeftNodeIdx].Position.To3D() + transform.position, 
                            Nodes[element.RightNodeIdx].Position.To3D() + transform.position);
        }
    }
}