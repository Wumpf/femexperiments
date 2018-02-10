using System;
using UnityEditor;
using System.Linq;

class ShapeEditor<TElementType> : Editor
    where TElementType : FEMElement2D
{
    public float AverageYoungModulus
    {
        get { return ((FEMShape2D<TElementType>)target).Elements.Average(x => x.YoungModulusGPa); }
        set
        {
            foreach (var e in ((FEMShape2D<TElementType>)target).Elements)
                e.YoungModulusGPa = value;
        }
    }
    public float AverageDensity
    {
        get { return ((FEMShape2D<TElementType>)target).Elements.Average(x => x.Density); }
        set
        {
            foreach (var e in ((FEMShape2D<TElementType>)target).Elements)
                e.Density = value;
        }
    }
    public float AverageDamping
    {
        get { return ((FEMShape2D<TElementType>)target).Elements.Average(x => x.DampingCoefficient); }
        set
        {
            foreach (var e in ((FEMShape2D<TElementType>)target).Elements)
                e.DampingCoefficient = value;
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