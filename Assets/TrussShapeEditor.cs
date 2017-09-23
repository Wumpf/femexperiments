using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(TrussShape))]
class TrussShapeEditor : Editor
{
    private int selectedNode = 0;

    public bool ShowLables { get; set; } = false;

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
            targetShape.Nodes.Add(new TrussShape.Node(mousePosition - targetShape.transform.position));
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
}