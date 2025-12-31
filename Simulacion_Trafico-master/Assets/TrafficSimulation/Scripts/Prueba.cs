using System.Diagnostics;
using UnityEngine;
using UnityEngine.AI;

public class NavMeshDebug : MonoBehaviour
{
    void Start()
    {
        UnityEngine.Debug.Log("NavMesh triangulos: " + NavMesh.CalculateTriangulation().vertices.Length);
    }
}

