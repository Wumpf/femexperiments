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

    [Serializable]
    public abstract class Element
    {
        [Tooltip("Thickness of the element in meters")]
        public float Thickness = 0.001f;
        
        [Tooltip("Describes stiffness of material. Rubber has 0.01-0.1, steel has 209")]
        public float YoungModulusGPa = 1.0f;

        public float YoungModulus => YoungModulusGPa * 1000000000;
        
        [Tooltip("kg per cubic meter. 1000 is water.")]
        public float Density = 1000.0f;

        public float DampingCoefficient = 50.0f;
    }
}