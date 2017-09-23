using UnityEngine;

public static class VectorExtension
{
    public static Vector3 To3D(this Vector2 v)
    {
        return new Vector3(v.x, v.y, 0.0f);
    }
    
    public static Vector2 To2D(this Vector3 v)
    {
        return new Vector2(v.x, v.y);
    }    
}
