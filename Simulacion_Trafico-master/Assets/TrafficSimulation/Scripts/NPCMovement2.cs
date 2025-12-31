//using System.Diagnostics;
//using UnityEngine;
//using UnityEngine.AI;
//using System.Collections;

//public class NPCMovement : MonoBehaviour
//{
//    public Transform[] destinos;
//    public float tiempoEsperaEnDestino = 2f;

//    private NavMeshAgent agente;
//    private int destinoActual = 0;
//    private bool listoParaMoverse = false;

//    void Start()
//    {
//        agente = GetComponent<NavMeshAgent>();
//        StartCoroutine(ColocarYActivarAgente());
//    }

//    IEnumerator ColocarYActivarAgente()
//    {
//        // Espera un frame para que Unity inicialice el NavMesh
//        yield return null;

//        NavMeshHit hit;
//        if (NavMesh.SamplePosition(transform.position, out hit, 2f, NavMesh.AllAreas))
//        {
//            agente.Warp(hit.position); // MUY IMPORTANTE usar Warp
//        }
//        else
//        {
//            UnityEngine.Debug.LogError("NPC fuera del NavMesh");
//            yield break;
//        }

//        // Esperar hasta que el agente esté realmente en el NavMesh
//        while (!agente.isOnNavMesh)
//        {
//            yield return null;
//        }

//        listoParaMoverse = true;
//        MoverANuevoDestino();
//    }

//    void MoverANuevoDestino()
//    {
//        if (!listoParaMoverse || destinos.Length == 0)
//            return;

//        agente.SetDestination(destinos[destinoActual].position);
//        destinoActual = (destinoActual + 1) % destinos.Length;
//        Invoke(nameof(MoverANuevoDestino), tiempoEsperaEnDestino);
//    }
//}



using System;
using System.Collections;
using System.Diagnostics;
using UnityEngine;
using UnityEngine.AI;

public class NPCMovement : MonoBehaviour
{
    public Transform[] destinos; // Asignar en Inspector los 3 destinos
    private int destinoActual;
    private NavMeshAgent agente;

    // Opciones de variabilidad
    public float velocidadMin = 1.5f;
    public float velocidadMax = 3.5f;
    public float delayMin = 0f;
    public float delayMax = 2f;

    void Start()
    {
        agente = GetComponent<NavMeshAgent>();

        // Velocidad aleatoria
        agente.speed = UnityEngine.Random.Range(velocidadMin, velocidadMax);

        // Elegir destino inicial aleatorio
        destinoActual = 0;

        // Iniciar con delay aleatorio
        float delay = UnityEngine.Random.Range(delayMin, delayMax);
        StartCoroutine(IniciarConDelay(delay));
    }

    IEnumerator IniciarConDelay(float delay)
    {
        yield return new WaitForSeconds(delay);

        if (!agente.isOnNavMesh)
        {
            UnityEngine.Debug.LogError(name + " NO está sobre el NavMesh");
            yield break;
        }

        MoverANuevoDestino();
    }

    void Update()
    {
        if (!agente.pathPending && agente.remainingDistance <= agente.stoppingDistance)
        {
            // Cuando llega al destino actual, pasa al siguiente
            destinoActual = (destinoActual + 1) % destinos.Length;
            MoverANuevoDestino();
        }
    }

    void MoverANuevoDestino()
    {
        if (destinos.Length == 0) return;
        agente.SetDestination(destinos[destinoActual].position);
    }
}
