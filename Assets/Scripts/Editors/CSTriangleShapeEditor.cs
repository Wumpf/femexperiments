using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra.Double;
using UnityEditor;
using UnityEngine;
using Random = UnityEngine.Random;

[CustomEditor(typeof(CSTriangleShape))]
 class CSTriangleShapeEditor : ShapeEditor<CSTriangleElement>
 {
     private bool showBlobGenerator = false;
     private int numBlobNodes = 10;
     
     public override void OnInspectorGUI()
     {
         base.OnInspectorGUI();
         
         EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);
 
         BlobGeneratorUI();
     }
 
     private void BlobGeneratorUI()
     {
         showBlobGenerator = EditorGUILayout.Foldout(showBlobGenerator, new GUIContent("Blob Generation"));
         if (showBlobGenerator)
         {
             numBlobNodes = EditorGUILayout.IntSlider(new GUIContent("Num Vertices"), numBlobNodes, 3, 100);

             if (GUILayout.Button(new GUIContent("Generate Blob")))
                 GenerateBlob();
         }
     }
 
     private void GenerateBlob()
     {
         var triangleShape = (CSTriangleShape) target;
         
         // cloud of points.
         var nodes = new List<Node>(numBlobNodes);
         for (int i = 0; i < numBlobNodes; ++i)
             nodes.Add(new Node(Random.insideUnitCircle));
         
         triangleShape.Nodes = nodes;
         ReTriangulate();
         EditorUtility.SetDirty(target);
     }

     private void ReTriangulate()
     {
         var triangleShape = (CSTriangleShape) target;
         
         var triangles = triangleShape.Elements = new List<CSTriangleElement>();
         var nodes = triangleShape.Nodes;
         
         // Random triangulation.
         // Create super triangle so we can iteratively build.
         const float superTriangleExtent = 1e10f;
         triangles.Add(new CSTriangleElement(triangleShape.Nodes.Count, triangleShape.Nodes.Count + 1, triangleShape.Nodes.Count + 2));
         nodes.AddRange(new Node[]
         {
             new Node(new Vector2(0, superTriangleExtent)),
             new Node(new Vector2(-superTriangleExtent, -superTriangleExtent)),
             new Node(new Vector2(superTriangleExtent, -superTriangleExtent)),
         });
         
         // Add points by splitting triangle into two.
         var edges = new HashSet<Tuple<int, int>>();
         for (int i = 0; i < nodes.Count - 3; ++i)
         {
             var badTriangles = triangles.Where(tri => IsPointInTriangleCircumcircle(nodes[i].Position, nodes[tri.NodeIdxA].Position, nodes[tri.NodeIdxB].Position, nodes[tri.NodeIdxC].Position)).ToList();
             foreach (var tri in badTriangles)
             {
                 Tuple<int, int> edge;
                 
                 edge = tri.NodeIdxA < tri.NodeIdxB ? new Tuple<int, int>(tri.NodeIdxA, tri.NodeIdxB) : new Tuple<int, int>(tri.NodeIdxB, tri.NodeIdxA);
                 if (!edges.Remove(edge))
                     edges.Add(edge);

                 edge = tri.NodeIdxB < tri.NodeIdxC ? new Tuple<int, int>(tri.NodeIdxB, tri.NodeIdxC) : new Tuple<int, int>(tri.NodeIdxC, tri.NodeIdxB);
                 if (!edges.Remove(edge))
                     edges.Add(edge);
                 
                 edge = tri.NodeIdxC < tri.NodeIdxA ? new Tuple<int, int>(tri.NodeIdxC, tri.NodeIdxA) : new Tuple<int, int>(tri.NodeIdxA, tri.NodeIdxC);
                 if (!edges.Remove(edge))
                     edges.Add(edge);

                 triangles.Remove(tri);
             }

             foreach (var edge in edges)
             {
                 var newTri = new CSTriangleElement(edge.Item1, edge.Item2, i);
                 newTri.FixPointOrder(nodes);
                 triangles.Add(newTri);
             }
             
             edges.Clear();
         }
         
         // remove super triangle.
         nodes.RemoveRange(nodes.Count - 3, 3);
         triangleShape.Elements = triangles.Where(t => t.NodeIdxA < nodes.Count && t.NodeIdxB < nodes.Count && t.NodeIdxC < nodes.Count).ToList();
     }

     private bool IsPointInTriangleCircumcircle(Vector2 point, Vector2 trianglePointA, Vector2 trianglePointB, Vector2 trianglePointC)
     {
         var m = new Matrix4x4();
         m.m00 = trianglePointA.x;
         m.m10 = trianglePointB.x;
         m.m20 = trianglePointC.x;
         m.m30 = point.x;
         m.m01 = trianglePointA.y;
         m.m11 = trianglePointB.y;
         m.m21 = trianglePointC.y;
         m.m31 = point.y;
         m.m02 = trianglePointA.sqrMagnitude;
         m.m12 = trianglePointB.sqrMagnitude;
         m.m22 = trianglePointC.sqrMagnitude;
         m.m32 = point.sqrMagnitude;
         m.m03 = 1.0f;
         m.m13 = 1.0f;
         m.m23 = 1.0f;
         m.m33 = 1.0f; 
         return m.determinant > 0.0f;
     }
 }