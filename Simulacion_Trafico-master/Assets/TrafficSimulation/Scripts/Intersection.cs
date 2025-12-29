using System.Collections.Generic;
using UnityEngine;

namespace TrafficSimulation
{
    public enum IntersectionType
    {
        STOP,
        TRAFFIC_LIGHT
    }

    public class Intersection : MonoBehaviour
    {
        [Header("Configuración General")]
        public IntersectionType intersectionType;
        public int id;

        [Header("Configuración STOP")]
        public List<Segment> prioritySegments; // Segmentos que NO paran en el STOP

        [Header("Semáforos Inteligentes (AI Parameters)")]
        public float minGreenTime = 5.0f;
        public float maxWaitTimeSafety = 20.0f;
        public float urgencyCoefficient = 2.0f;

        [Header("Estructura de Vías (Semáforos)")]
        public List<Segment> lightsNbr1;
        public List<Segment> lightsNbr2;

        // --- VARIABLES INTERNAS ---

        // Para lógica STOP (Restauradas)
        private List<GameObject> vehiclesQueue = new List<GameObject>();
        private List<GameObject> vehiclesInIntersection = new List<GameObject>();

        // Para lógica SEMÁFORO (IA)
        private Dictionary<int, TrafficSensor> sensors = new Dictionary<int, TrafficSensor>();
        [HideInInspector] public int currentRedLightsGroup = 1;
        private float lastLightSwitchTime;

        void Start()
        {
            // Inicialización de listas
            vehiclesQueue = new List<GameObject>();
            vehiclesInIntersection = new List<GameObject>();

            if (intersectionType == IntersectionType.TRAFFIC_LIGHT)
            {
                InitializeSensors(lightsNbr1);
                InitializeSensors(lightsNbr2);
                lastLightSwitchTime = Time.time;
            }
        }

        void InitializeSensors(List<Segment> segmentGroup)
        {
            foreach (var seg in segmentGroup)
            {
                if (!sensors.ContainsKey(seg.id))
                {
                    sensors.Add(seg.id, new TrafficSensor(seg));
                }
            }
        }

        void Update()
        {
            // Solo ejecutamos lógica de IA si es un semáforo
            if (intersectionType == IntersectionType.TRAFFIC_LIGHT)
            {
                UpdateSensors();
                DecideNextLightState();
            }
        }

        // --- SISTEMA DE DETECCIÓN (TRIGGERS) ---

        void OnTriggerEnter(Collider _other)
        {
            // Evitamos procesar si el objeto no es un vehículo o acabamos de empezar
            if (_other.tag != "AutonomousVehicle" || Time.timeSinceLevelLoad < .5f) return;

            // Evitamos doble procesamiento si el coche ya está registrado
            if (IsAlreadyInIntersection(_other.gameObject)) return;

            VehicleAI vehicleAI = _other.GetComponent<VehicleAI>();
            int segmentId = vehicleAI.GetSegmentVehicleIsIn();

            if (intersectionType == IntersectionType.TRAFFIC_LIGHT)
            {
                // Registro en sensores IA
                if (sensors.ContainsKey(segmentId))
                {
                    sensors[segmentId].DetectVehicle(_other.gameObject);
                }
                TriggerLight(_other.gameObject, segmentId);
            }
            else if (intersectionType == IntersectionType.STOP)
            {
                TriggerStop(_other.gameObject, segmentId);
            }
        }

        void OnTriggerExit(Collider _other)
        {
            if (_other.tag != "AutonomousVehicle") return;

            VehicleAI vehicleAI = _other.GetComponent<VehicleAI>();

            // Eliminar del sensor IA si corresponde
            foreach (var sensor in sensors.Values)
            {
                sensor.RemoveVehicle(_other.gameObject);
            }

            if (intersectionType == IntersectionType.STOP)
            {
                ExitStop(_other.gameObject);
            }
            else
            {
                ExitLight(_other.gameObject);
            }
        }

        // --- LÓGICA RESTAURADA: STOP ---

        void TriggerStop(GameObject _vehicle, int vehicleSegment)
        {
            VehicleAI vehicleAI = _vehicle.GetComponent<VehicleAI>();

            // Si vengo de un carril prioritario, paso (pero aviso que estoy dentro)
            if (IsPrioritySegment(vehicleSegment))
            {
                vehicleAI.vehicleStatus = Status.SLOW_DOWN; // Pasa despacio
                vehiclesInIntersection.Add(_vehicle);
            }
            else
            {
                // Si NO tengo prioridad...
                // Si hay alguien esperando (cola) O hay alguien cruzando -> ME PARO
                if (vehiclesQueue.Count > 0 || vehiclesInIntersection.Count > 0)
                {
                    vehicleAI.vehicleStatus = Status.STOP;
                    vehiclesQueue.Add(_vehicle);
                }
                else
                {
                    // Si está vacío -> PASO (soy el primero)
                    vehicleAI.vehicleStatus = Status.SLOW_DOWN;
                    vehiclesInIntersection.Add(_vehicle);
                }
            }
        }

        void ExitStop(GameObject _vehicle)
        {
            // El coche que sale se pone en GO
            _vehicle.GetComponent<VehicleAI>().vehicleStatus = Status.GO;

            // Lo sacamos de las listas de control
            vehiclesInIntersection.Remove(_vehicle);
            vehiclesQueue.Remove(_vehicle);

            // GESTIÓN DE LA COLA:
            // Si hay coches esperando Y nadie cruzando ahora mismo...
            if (vehiclesQueue.Count > 0 && vehiclesInIntersection.Count == 0)
            {
                // Damos paso al siguiente de la cola
                vehiclesQueue[0].GetComponent<VehicleAI>().vehicleStatus = Status.GO;
                // Nota: No lo añadimos a 'vehiclesInIntersection' aquí, 
                // ya que técnicamente está saliendo de la cola, pero la cola actúa como bloqueo para el tercero.
            }
        }

        // --- LÓGICA SEMÁFORO INTELIGENTE (IA) ---

        void UpdateSensors()
        {
            float dt = Time.deltaTime;
            foreach (var sensor in sensors.Values) sensor.UpdateSensor(dt);
        }

        void DecideNextLightState()
        {
            if (Time.time - lastLightSwitchTime < minGreenTime) return;

            float weightGroup1 = GetGroupWeight(lightsNbr1);
            float weightGroup2 = GetGroupWeight(lightsNbr2);

            if (currentRedLightsGroup == 1)
            {
                if (weightGroup1 > weightGroup2) SwitchLights();
            }
            else
            {
                if (weightGroup2 > weightGroup1) SwitchLights();
            }
        }

        float GetGroupWeight(List<Segment> group)
        {
            float totalWeight = 0;
            foreach (var seg in group)
            {
                if (sensors.ContainsKey(seg.id))
                    totalWeight += sensors[seg.id].GetPriorityWeight(urgencyCoefficient);
            }
            return totalWeight;
        }

        public void SwitchLights()
        { // 'public' para que el Brain pueda llamarlo
            currentRedLightsGroup = (currentRedLightsGroup == 1) ? 2 : 1;
            lastLightSwitchTime = Time.time;
            MoveVehiclesQueue();
        }

        void TriggerLight(GameObject _vehicle, int vehicleSegment)
        {
            VehicleAI vehicleAI = _vehicle.GetComponent<VehicleAI>();
            if (IsRedLightSegment(vehicleSegment))
            {
                vehicleAI.vehicleStatus = Status.STOP;
            }
            else
            {
                vehicleAI.vehicleStatus = Status.GO;
                if (sensors.ContainsKey(vehicleSegment)) sensors[vehicleSegment].RemoveVehicle(_vehicle);
            }
        }

        void MoveVehiclesQueue()
        {
            List<Segment> greenSegments = (currentRedLightsGroup == 1) ? lightsNbr2 : lightsNbr1;
            foreach (var seg in greenSegments)
            {
                if (sensors.ContainsKey(seg.id))
                {
                    foreach (GameObject v in sensors[seg.id].waitingVehicles)
                    {
                        if (v != null) v.GetComponent<VehicleAI>().vehicleStatus = Status.GO;
                    }
                }
            }
        }

        void ExitLight(GameObject _vehicle)
        {
            if (_vehicle != null) _vehicle.GetComponent<VehicleAI>().vehicleStatus = Status.GO;
        }

        // --- UTILIDADES ---

        bool IsPrioritySegment(int _vehicleSegment)
        {
            foreach (Segment s in prioritySegments) if (_vehicleSegment == s.id) return true;
            return false;
        }

        bool IsRedLightSegment(int _vehicleSegment)
        {
            List<Segment> redGroup = (currentRedLightsGroup == 1) ? lightsNbr1 : lightsNbr2;
            foreach (Segment segment in redGroup)
            {
                if (segment.id == _vehicleSegment) return true;
            }
            return false;
        }

        bool IsAlreadyInIntersection(GameObject _target)
        {
            // Verifica si el coche ya está siendo gestionado para no duplicarlo en las listas
            foreach (GameObject vehicle in vehiclesInIntersection)
            {
                if (vehicle != null && vehicle.GetInstanceID() == _target.GetInstanceID()) return true;
            }
            foreach (GameObject vehicle in vehiclesQueue)
            {
                if (vehicle != null && vehicle.GetInstanceID() == _target.GetInstanceID()) return true;
            }
            return false;
        }

        // Métodos de guardado vacíos (opcional implementarlos si necesitas save/load)
        public void SaveIntersectionStatus() { }
        public void ResumeIntersectionStatus() { }
    }
}
