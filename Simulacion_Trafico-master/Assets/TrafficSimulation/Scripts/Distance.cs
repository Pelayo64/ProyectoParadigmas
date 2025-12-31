//using System.Diagnostics;
//using UnityEngine;
//using UnityEngine.AI;

//public class NPCSpawner : MonoBehaviour
//{
//    public GameObject npcPrefab;

//    void Start()
//    {
//        Vector3 spawnPos = transform.position; // posición inicial
//        NavMeshHit hit;

//        // Busca el NavMesh más cercano en un radio de 10 unidades
//        if (NavMesh.SamplePosition(spawnPos, out hit, 10f, NavMesh.AllAreas))
//        {
//            GameObject npc = Instantiate(npcPrefab, hit.position, Quaternion.identity);
//            UnityEngine.Debug.Log(npc.name + " colocado correctamente sobre el NavMesh.");
//        }
//        else
//        {
//            UnityEngine.Debug.LogError("No se pudo colocar NPC sobre el NavMesh. Aumenta el radio o revisa la NavMesh.");
//        }
//    }
//}


using UnityEngine;
using UnityEngine.AI;

public class NPCSpawner : MonoBehaviour
{
    public GameObject npcPrefab;
    public Transform[] destinos;
    public int cantidadNPC = 5;

    void Start()
    {
        for (int i = 0; i < cantidadNPC; i++)
        {
            Vector3 spawnPos = transform.position;
            NavMeshHit hit;
            if (NavMesh.SamplePosition(spawnPos, out hit, 10f, NavMesh.AllAreas))
            {
                GameObject npc = Instantiate(npcPrefab, hit.position, Quaternion.identity);
                // Asignar los destinos al NPC instanciado
                NPCMovement movimiento = npc.GetComponent<NPCMovement>();
                movimiento.destinos = destinos;
            }
        }
    }
}
