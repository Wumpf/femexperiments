using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEditor;
using UnityEngine;

public abstract class FEMShape2D
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
}