using System;
using UnityEditor;
using UnityEngine;
using System.Linq;

[CustomEditor(typeof(TrussShape))]
class TrussShapeEditor : Editor
{
    private int selectedNode = 0;

    public bool ShowLables { get; set; } = false;
    
    public float AverageYoungModulus
    {
        get { return ((TrussShape)target).Elements.Average(x => x.YoungModulusGPa); }
        set
        {
            foreach (var e in ((TrussShape)target).Elements)
                e.YoungModulusGPa = value;
        }
    }
    public float AverageDensity
    {
        get { return ((TrussShape)target).Elements.Average(x => x.Density); }
        set
        {
            foreach (var e in ((TrussShape)target).Elements)
                e.Density = value;
        }
    }
    public float AverageDamping
    {
        get { return ((TrussShape)target).Elements.Average(x => x.DampingCoefficient); }
        set
        {
            foreach (var e in ((TrussShape)target).Elements)
                e.DampingCoefficient = value;
        }
    }
    

    protected virtual void OnSceneGUI()
    {
        TrussShape targetShape = (TrussShape)target;
        if (targetShape == null)
        {
            return;
        }

        GUIStyle style = new GUIStyle();
        style.normal.textColor = Color.green;
        
        // Adding nodes.
        if (Event.current.type == EventType.KeyDown && Event.current.keyCode == KeyCode.Space)
        {
            var mousePosition = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).origin;
            Debug.Log(mousePosition);
            Debug.Log(mousePosition - targetShape.transform.position);
            EditorGUI.BeginChangeCheck();
            targetShape.Nodes.Add(new Node(mousePosition - targetShape.transform.position));
            EditorGUI.EndChangeCheck();
            Undo.RecordObject(targetShape, "Add truss node.");
        }

        // Nodes.
        for (int i = 0; i < targetShape.Nodes.Count; ++i)
        {
            var node = targetShape.Nodes[i];
            var pos = targetShape.GetNodeWorldPosition(i);

            if (i == selectedNode)
                Handles.color = Color.red;
            else
                Handles.color = Color.white;
            
            var buttonSize = HandleUtility.GetHandleSize(pos) * 0.08f;
            Handles.CapFunction capfunc = Handles.SphereHandleCap;
            if (node.Fixed)
                capfunc = Handles.DotHandleCap;
            if (Handles.Button(pos, Quaternion.identity, buttonSize, buttonSize, capfunc))
            {
                if (Event.current.shift)
                {
                    EditorGUI.BeginChangeCheck();
                    targetShape.Elements.Add(new TrussShape.Element(selectedNode, i));
                    EditorGUI.EndChangeCheck();
                    Undo.RecordObject(targetShape, "Add truss element.");
                }
                selectedNode = i;
            }
                
            
            // Moving selected node.
            if (i == selectedNode)
            {
                EditorGUI.BeginChangeCheck();
                Vector3 newTargetPosition = Handles.PositionHandle(targetShape.GetNodeWorldPosition(i), Quaternion.identity);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(targetShape, "Change TrussShape node position");
                    newTargetPosition -= targetShape.transform.position;
                    node.Position = new Vector2(newTargetPosition.x, newTargetPosition.y);
                    targetShape.Nodes[i] = node;
                }
            }
        }
    }

    private void ExposeAverageValue(Func<float> get, Action<float> set, string name)
    {
        float average = get();
        float newAverage = EditorGUILayout.FloatField(name, average);
        if (newAverage != average)
        {
            EditorGUI.BeginChangeCheck();
            set(newAverage);
            EditorGUI.EndChangeCheck();
            Undo.RecordObject(target, $"Change {name}.");
        }
    }

    private bool averageFoldout = true;

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        averageFoldout = EditorGUILayout.Foldout(averageFoldout, "Average Element values");
        if (averageFoldout)
        {
            using (var scope = new EditorGUI.IndentLevelScope(1))
            {
                ExposeAverageValue(() => AverageYoungModulus, x => AverageYoungModulus = x, "AverageYoungModulus");
                ExposeAverageValue(() => AverageDensity, x => AverageDensity = x, "AverageDensity");
                ExposeAverageValue(() => AverageDamping, x => AverageDamping = x, "AverageDamping");
            }
        }
    }
}