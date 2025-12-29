using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq; // Necesario para Linq

namespace TrafficSimulation {
    
    [RequireComponent(typeof(Intersection))]
    public class TrafficAI_Brain : MonoBehaviour {
        [Header("Configuración Q-Learning")]
        public float learningRate = 0.1f;   // Alpha: Qué tan rápido olvida lo viejo
        public float discountFactor = 0.9f; // Gamma: Importancia del futuro
        public float explorationRate = 1.0f; // Epsilon: 1 = 100% Aleatorio (Explorar), 0 = 100% Cerebro (Explotar)
        public float explorationDecay = 0.995f; // Cuánto baja la exploración cada paso
        public float minExploration = 0.01f;
        
        [Header("Tiempos")]
        public float decisionInterval = 5.0f; // Cada cuánto toma una decisión
        
        // Tabla Q: [Estado_Grupo1, Estado_Grupo2, Acción] -> Valor Esperado
        // Estados: 0=Vacío, 1=Poco, 2=Medio, 3=Mucho
        private float[,,] qTable = new float[4, 4, 2]; 
        
        private Intersection intersection;
        private float timer = 0;
        private int lastState1 = 0;
        private int lastState2 = 0;
        private int lastAction = 0; // 0=Mantener, 1=Cambiar

        void Start() {
            intersection = GetComponent<Intersection>();
            // Desactivamos la lógica interna de la intersección para tomar el control nosotros
            // (Asegúrate de que Intersection no cambie luces por su cuenta si este script está activo)
            // Una forma simple es poner 'intersectionType' en STOP en el inspector para que Intersection no use su Update, 
            // pero nosotros llamaremos a sus métodos manualmente.
            
            LoadBrain();
        }

        void Update() {
            timer += Time.deltaTime;
            
            if (timer >= decisionInterval) {
                TrainAndAct();
                timer = 0;
            }
        }

        void TrainAndAct() {
            // 1. OBSERVAR: Obtener el estado actual (Nivel de tráfico discretizado)
            int currentState1 = DiscretizeTraffic(intersection.lightsNbr1);
            int currentState2 = DiscretizeTraffic(intersection.lightsNbr2);

            // 2. RECOMPENSA: Calcular qué tan bien lo hicimos desde la última vez
            // La recompensa es negativa: queremos minimizar el tiempo de espera total.
            // R = -(Suma de tiempos de espera de todos los carriles)
            float currentWaitTime = GetTotalWaitTime();
            float reward = -currentWaitTime;

            // 3. APRENDER: Actualizar la Tabla Q (Ecuación de Bellman)
            // Q(s,a) = Q(s,a) + alpha * (R + gamma * max(Q(s', all_a)) - Q(s,a))
            float oldQ = qTable[lastState1, lastState2, lastAction];
            float maxFutureQ = Mathf.Max(qTable[currentState1, currentState2, 0], qTable[currentState1, currentState2, 1]);
            
            float newQ = oldQ + learningRate * (reward + discountFactor * maxFutureQ - oldQ);
            qTable[lastState1, lastState2, lastAction] = newQ;

            // 4. DECIDIR: Elegir nueva acción (Epsilon-Greedy)
            int nextAction = 0;
            if (UnityEngine.Random.value < explorationRate) {
                // Explorar: Acción aleatoria
                nextAction = UnityEngine.Random.Range(0, 2);
            } else {
                // Explotar: Mejor acción conocida
                float valAction0 = qTable[currentState1, currentState2, 0]; // Mantener
                float valAction1 = qTable[currentState1, currentState2, 1]; // Cambiar
                nextAction = (valAction1 > valAction0) ? 1 : 0;
            }

            // 5. ACTUAR
            if (nextAction == 1) {
                // Acción 1: Cambiar semáforo
                intersection.SwitchLights(); 
                // Nota: Asegúrate de que SwitchLights sea 'public' en Intersection.cs
            } 
            // Acción 0: Mantener luces (no hacemos nada)

            // 6. PREPARAR SIGUIENTE PASO
            lastState1 = currentState1;
            lastState2 = currentState2;
            lastAction = nextAction;

            // Decaer exploración (aprender menos, actuar más)
            if (explorationRate > minExploration) explorationRate *= explorationDecay;
            
            // Debug para ver progreso
            // UnityEngine.Debug.Log($"AI Log | Reward: {reward:F1} | Eps: {explorationRate:F3} | Q: {newQ:F1}");
        }

        // --- Herramientas Auxiliares ---

        // Convierte el tráfico continuo en niveles discretos para la tabla (0, 1, 2, 3)
        int DiscretizeTraffic(List<Segment> segments) {
            int count = 0;
            // Accedemos a los sensores de la intersección. 
            // NOTA: Esto asume que Intersection tiene un diccionario 'sensors' público o accesible.
            // Si no, contamos "a ojo" usando Raycasts o datos simples.
            // Aquí simularé un conteo rápido si no tienes acceso directo a los sensores privados:
            foreach(var seg in segments){
                // Un hack simple para contar: buscar coches cuya posición esté en este segmento y parados
                // Idealmente usarías intersection.GetTrafficCount(seg), pero lo haremos genérico:
                var vehicles = GameObject.FindGameObjectsWithTag("AutonomousVehicle");
                foreach(var v in vehicles){
                    var ai = v.GetComponent<VehicleAI>();
                    if(ai != null && ai.GetSegmentVehicleIsIn() == seg.id && ai.vehicleStatus == Status.STOP){
                        count++;
                    }
                }
            }

            if (count == 0) return 0;       // Vacío
            if (count <= 2) return 1;       // Bajo
            if (count <= 5) return 2;       // Medio
            return 3;                       // Alto
        }

        float GetTotalWaitTime() {
            // Suma simple de colas
            int totalCarsWaiting = 0;
            var vehicles = GameObject.FindGameObjectsWithTag("AutonomousVehicle");
            foreach(var v in vehicles){
                if(v.GetComponent<VehicleAI>().vehicleStatus == Status.STOP) totalCarsWaiting++;
            }
            return totalCarsWaiting * 1.0f; // Multiplicamos por 1 seg como unidad de coste
        }

        // --- Persistencia ---
        // Guardamos el cerebro al cerrar para no perder el entrenamiento
        void OnApplicationQuit() {
            SaveBrain();
        }

        void SaveBrain() {
            string data = "";
            for(int i=0; i<4; i++)
                for(int j=0; j<4; j++)
                    for(int k=0; k<2; k++)
                        data += qTable[i,j,k] + ",";
            PlayerPrefs.SetString("TrafficBrain_" + intersection.id, data);
            PlayerPrefs.Save();
        }

        void LoadBrain() {
            string key = "TrafficBrain_" + intersection.id;
            if(PlayerPrefs.HasKey(key)) {
                string[] data = PlayerPrefs.GetString(key).Split(',');
                int idx = 0;
                for(int i=0; i<4; i++)
                    for(int j=0; j<4; j++)
                        for(int k=0; k<2; k++)
                            if(idx < data.Length && !string.IsNullOrEmpty(data[idx]))
                                qTable[i,j,k] = float.Parse(data[idx++]);
            }
        }
    }
}