using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace TrafficSimulation
{

    public struct Target
    {
        public int segment;
        public int waypoint;
    }

    public enum Status
    {
        GO,
        STOP,
        SLOW_DOWN
    }

    public class VehicleAI : MonoBehaviour
    {
        [Header("Traffic System")]
        public TrafficSystem trafficSystem;
        public float waypointThresh = 6;

        [Header("Comportamiento Humano")]
        public float minReactionTime = 0.5f;
        public float maxReactionTime = 1.0f;
        public float accelerationSmoothness = 2.0f;

        [Header("Sensores Inteligentes (LIDAR)")]
        public Transform raycastAnchor;
        public float raycastLength = 12f;
        public int raySpacing = 2; // Espaciado en grados
        public int raysNumber = 6;
        public float emergencyBrakeThresh = 3.5f;
        public float slowDownThresh = 8f;

        // Estado interno
        private Status _vehicleStatus = Status.GO;
        public Status vehicleStatus
        {
            get { return _vehicleStatus; }
            set
            {
                if (_vehicleStatus == Status.STOP && value == Status.GO)
                {
                    StartCoroutine(ApplyReactionDelay());
                }
                else
                {
                    _vehicleStatus = value;
                }
            }
        }

        private bool isReacting = false;
        private float currentThrottle = 0f;
        private float timeStopped = 0f; // Para detectar bloqueos

        private WheelDrive wheelDrive;
        private float initMaxSpeed = 0;
        private int pastTargetSegment = -1;
        private Target currentTarget;
        private Target futureTarget;

        void Start()
        {
            wheelDrive = this.GetComponent<WheelDrive>();
            if (trafficSystem == null) return;
            initMaxSpeed = wheelDrive.maxSpeed;
            SetWaypointVehicleIsOn();
        }

        void Update()
        {
            if (trafficSystem == null) return;
            WaypointChecker();
            MoveVehicle();
        }

        IEnumerator ApplyReactionDelay()
        {
            isReacting = true;
            float delay = UnityEngine.Random.Range(minReactionTime, maxReactionTime);
            yield return new WaitForSeconds(delay);
            _vehicleStatus = Status.GO;
            isReacting = false;
        }

        void MoveVehicle()
        {
            float targetAcc = 0;
            float targetBrake = 0;
            float steering = 0;

            // Restablecer velocidad base
            wheelDrive.maxSpeed = initMaxSpeed;

            // 1. CÁLCULO DE DIRECCIÓN (STEERING)
            Transform targetTransform = trafficSystem.segments[currentTarget.segment].waypoints[currentTarget.waypoint].transform;
            Transform futureTargetTransform = trafficSystem.segments[futureTarget.segment].waypoints[futureTarget.waypoint].transform;
            Vector3 futureVel = futureTargetTransform.position - targetTransform.position;
            float futureSteering = Mathf.Clamp(this.transform.InverseTransformDirection(futureVel.normalized).x, -1, 1);

            // Dirección actual hacia el waypoint
            Vector3 desiredVel = targetTransform.position - this.transform.position;
            steering = Mathf.Clamp(this.transform.InverseTransformDirection(desiredVel.normalized).x, -1f, 1f);

            // 2. GESTIÓN DE ESTADOS (Stop / Go)
            bool isStopped = (_vehicleStatus == Status.STOP || isReacting);

            // Permitir giro a la derecha en rojo si es seguro
            if (isStopped && CheckRightTurnOnRed(targetTransform, futureTargetTransform))
            {
                isStopped = false;
            }

            if (isStopped)
            {
                targetAcc = 0;
                targetBrake = 1;
            }
            else
            {
                // Estado por defecto: Acelerar
                targetAcc = 1;
                targetBrake = 0;

                // Reducir velocidad si vamos a girar mucho (curvas)
                if (Mathf.Abs(steering) > 0.3f || Mathf.Abs(futureSteering) > 0.3f)
                {
                    wheelDrive.maxSpeed = Mathf.Min(wheelDrive.maxSpeed, wheelDrive.steeringSpeedMax);
                }

                // 3. DETECCIÓN DE OBSTÁCULOS REFINADA
                float hitDist;
                // Pasamos el steering actual para ajustar los rayos dinámicamente
                GameObject obstacle = GetDetectedObstacles(steering, out hitDist);

                if (obstacle != null)
                {
                    // Si detectamos algo, reseteamos contador de bloqueo para gestionarlo con física
                    timeStopped = 0;

                    float dangerFactor = 1f - Mathf.Clamp01((hitDist - emergencyBrakeThresh) / (slowDownThresh - emergencyBrakeThresh));

                    if (hitDist < emergencyBrakeThresh)
                    {
                        // Freno de emergencia
                        targetAcc = 0;
                        targetBrake = 1;
                        wheelDrive.maxSpeed = 0;
                    }
                    else if (hitDist < slowDownThresh)
                    {
                        // Aproximación suave
                        targetAcc = Mathf.Lerp(1f, 0f, dangerFactor * 1.5f);
                        targetBrake = Mathf.Lerp(0f, 0.5f, dangerFactor);

                        // Adaptar velocidad al coche de delante
                        WheelDrive otherCar = obstacle.GetComponent<WheelDrive>();
                        if (otherCar != null)
                        {
                            float otherSpeed = otherCar.GetSpeedUnit(otherCar.GetComponent<Rigidbody>().linearVelocity.magnitude);
                            wheelDrive.maxSpeed = Mathf.Min(wheelDrive.maxSpeed, otherSpeed);
                        }
                    }
                }
                else
                {
                    // SISTEMA ANTI-BLOQUEO:
                    // Si no hay obstáculo detectado pero el coche apenas se mueve (quizás por física o roces)
                    if (wheelDrive.GetComponent<Rigidbody>().linearVelocity.magnitude < 1f)
                    {
                        timeStopped += Time.deltaTime;
                        if (timeStopped > 2.0f)
                        {
                            // Si lleva 2 segundos "atascado" sin obstáculo delante, forzamos un empujón
                            targetAcc = 1f;
                        }
                    }
                    else
                    {
                        timeStopped = 0;
                    }
                }
            }

            // Aplicar suavizado al acelerador para simular peso
            currentThrottle = Mathf.MoveTowards(currentThrottle, targetAcc, Time.deltaTime * accelerationSmoothness);
            wheelDrive.Move(currentThrottle, steering, targetBrake);
        }

        // --- SISTEMA DE SENSORES INTELIGENTE ---
        GameObject GetDetectedObstacles(float steeringAngle, out float _hitDist)
        {
            GameObject closestObstacle = null;
            float minDist = 1000f;
            _hitDist = -1f;

            // Ajuste dinámico: Si giramos mucho, estrechamos el abanico de rayos para no ver coches aparcados a los lados
            float dynamicSpacing = (Mathf.Abs(steeringAngle) > 0.4f) ? raySpacing * 0.5f : raySpacing;
            float initRay = (raysNumber / 2f) * dynamicSpacing;

            for (float a = -initRay; a <= initRay; a += dynamicSpacing)
            {
                // Giramos el rayo según el ángulo 'a'
                Vector3 rayDir = Quaternion.Euler(0, a, 0) * this.transform.forward;

                RaycastHit hit;
                // Usamos LayerMask para ver solo vehículos
                int layerMask = 1 << LayerMask.NameToLayer("AutonomousVehicle");

                // Dibujo debug para ver qué está pasando en la escena
                UnityEngine.Debug.DrawRay(raycastAnchor.position, rayDir * raycastLength, new Color(1, 0, 0, 0.3f));

                if (Physics.Raycast(raycastAnchor.position, rayDir, out hit, raycastLength, layerMask))
                {
                    GameObject obj = hit.collider.gameObject;

                    // --- FILTRO DE INTELIGENCIA ---
                    // ¿Es este obstáculo una amenaza real?
                    if (IsIgnorableObstacle(obj, hit.point))
                    {
                        UnityEngine.Debug.DrawLine(raycastAnchor.position, hit.point, Color.green); // Debug: Ignorado
                        continue; // Saltamos este rayo, no cuenta como obstáculo
                    }

                    // Si no es ignorable, calculamos distancia
                    float dist = Vector3.Distance(this.transform.position, obj.transform.position);
                    if (dist < minDist)
                    {
                        minDist = dist;
                        closestObstacle = obj;
                        // Si encontramos algo crítico, paramos de buscar
                        if (minDist < emergencyBrakeThresh) break;
                    }
                }
            }

            _hitDist = (minDist == 1000f) ? -1f : minDist;
            return closestObstacle;
        }

        bool IsIgnorableObstacle(GameObject obj, Vector3 hitPoint)
        {
            // Regla 1: No soy yo mismo
            if (obj == this.gameObject) return true;

            // Regla 2: Análisis de dirección (Oncoming Traffic)
            float dot = Vector3.Dot(this.transform.forward, obj.transform.forward);

            // Si el dot es negativo (< -0.25), el coche viene de frente o está muy cruzado
            if (dot < -0.25f)
            {
                // Calculamos su posición lateral relativa a mí
                Vector3 localPos = this.transform.InverseTransformPoint(obj.transform.position);

                // Si viene de frente Y está desplazado lateralmente más de 1.2m (mitad de un carril aprox)
                // SIGNIFICA: Viene por el carril contrario -> IGNORAR
                if (Mathf.Abs(localPos.x) > 1.2f)
                {
                    return true;
                }

                // Excepción: Si viene de frente pero está MUY cerca (< 3m), no lo ignoramos por si acaso (choque frontal inminente)
                if (Vector3.Distance(this.transform.position, hitPoint) < 3.0f)
                {
                    return false;
                }
            }

            return false; // Es una amenaza válida (coche delante en mi carril)
        }

        // --- LÓGICA DE GIRO A LA DERECHA (Mantenida y pulida) ---
        bool CheckRightTurnOnRed(Transform currentWP, Transform nextWP)
        {
            Vector3 directionToNext = (nextWP.position - currentWP.position).normalized;
            float angle = Vector3.SignedAngle(this.transform.forward, directionToNext, Vector3.up);

            // Detectar giro a la derecha (> 30 grados)
            if (angle > 30f)
            {
                if (!IsTrafficIncomingFromLeft(nextWP))
                {
                    return true;
                }
            }
            return false;
        }

        bool IsTrafficIncomingFromLeft(Transform targetMergePoint)
        {
            Collider[] hits = Physics.OverlapSphere(targetMergePoint.position, 6.0f); // Radio aumentado ligeramente para seguridad
            foreach (var hit in hits)
            {
                if (hit.gameObject != this.gameObject && hit.CompareTag("AutonomousVehicle"))
                {
                    // Verificar velocidad del coche que viene: Si está parado, no cuenta como tráfico entrante peligroso
                    Rigidbody rb = hit.GetComponent<Rigidbody>();
                    if (rb != null && rb.linearVelocity.magnitude > 1.0f) return true;
                }
            }
            return false;
        }

        // --- SISTEMA WAYPOINTS (Standard) ---
        void WaypointChecker()
        {
            GameObject waypoint = trafficSystem.segments[currentTarget.segment].waypoints[currentTarget.waypoint].gameObject;
            Vector3 wpDist = this.transform.InverseTransformPoint(new Vector3(waypoint.transform.position.x, this.transform.position.y, waypoint.transform.position.z));

            if (wpDist.magnitude < waypointThresh)
            {
                currentTarget.waypoint++;
                if (currentTarget.waypoint >= trafficSystem.segments[currentTarget.segment].waypoints.Count)
                {
                    pastTargetSegment = currentTarget.segment;
                    currentTarget.segment = futureTarget.segment;
                    currentTarget.waypoint = 0;
                }
                futureTarget.waypoint = currentTarget.waypoint + 1;
                if (futureTarget.waypoint >= trafficSystem.segments[currentTarget.segment].waypoints.Count)
                {
                    futureTarget.waypoint = 0;
                    futureTarget.segment = GetNextSegmentId();
                }
            }
        }

        void SetWaypointVehicleIsOn()
        {
            foreach (Segment segment in trafficSystem.segments)
            {
                if (segment.IsOnSegment(this.transform.position))
                {
                    currentTarget.segment = segment.id;
                    float minDist = float.MaxValue;
                    for (int j = 0; j < trafficSystem.segments[currentTarget.segment].waypoints.Count; j++)
                    {
                        float d = Vector3.Distance(this.transform.position, trafficSystem.segments[currentTarget.segment].waypoints[j].transform.position);
                        Vector3 lSpace = this.transform.InverseTransformPoint(trafficSystem.segments[currentTarget.segment].waypoints[j].transform.position);
                        if (d < minDist && lSpace.z > 0)
                        {
                            minDist = d;
                            currentTarget.waypoint = j;
                        }
                    }
                    break;
                }
            }
            futureTarget.waypoint = currentTarget.waypoint + 1;
            futureTarget.segment = currentTarget.segment;
            if (futureTarget.waypoint >= trafficSystem.segments[currentTarget.segment].waypoints.Count)
            {
                futureTarget.waypoint = 0;
                futureTarget.segment = GetNextSegmentId();
            }
        }

        int GetNextSegmentId()
        {
            if (trafficSystem.segments[currentTarget.segment].nextSegments.Count == 0) return 0;
            int c = UnityEngine.Random.Range(0, trafficSystem.segments[currentTarget.segment].nextSegments.Count);
            return trafficSystem.segments[currentTarget.segment].nextSegments[c].id;
        }

        public int GetSegmentVehicleIsIn()
        {
            int vehicleSegment = currentTarget.segment;
            bool isOnSegment = trafficSystem.segments[vehicleSegment].IsOnSegment(this.transform.position);
            if (!isOnSegment)
            {
                bool isOnPSegement = trafficSystem.segments[pastTargetSegment].IsOnSegment(this.transform.position);
                if (isOnPSegement)
                    vehicleSegment = pastTargetSegment;
            }
            return vehicleSegment;
        }
    }
}