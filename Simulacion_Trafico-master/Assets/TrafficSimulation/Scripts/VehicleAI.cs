using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

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

        [Header("Física y Comportamiento")]
        public float minReactionTime = 0.5f;
        public float maxReactionTime = 1.0f;
        public float accelerationSmoothness = 2.0f;
        public float brakingSmoothness = 3.0f;

        [Header("Sensores (LIDAR)")]
        public Transform raycastAnchor;
        public float raycastLength = 16f;
        public int raysNumber = 7;
        public float emergencyBrakeThresh = 4.0f;
        public float slowDownThresh = 10.0f;

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
        private float currentBrake = 0f;

        // Anti-Bloqueo
        private float stuckTimer = 0f;
        private bool isStuck = false;

        // Variables de Navegación
        private WheelDrive wheelDrive;
        private float initMaxSpeed = 0;

        // --- CORRECCIÓN DEL ERROR ---
        private int pastTargetSegment = -1; // Variable reintroducida

        private Target currentTarget;
        private Target futureTarget;
        private Rigidbody rb;

        void Start()
        {
            wheelDrive = this.GetComponent<WheelDrive>();
            rb = this.GetComponent<Rigidbody>();

            if (trafficSystem == null) return;
            initMaxSpeed = wheelDrive.maxSpeed;
            SetWaypointVehicleIsOn();
        }

        void Update()
        {
            if (trafficSystem == null) return;

            CheckStuckCondition();
            WaypointChecker();
            MoveVehicle();
        }

        // --- LÓGICA DE RE-ROUTING (CAMBIO DE RUTA) ---
        void CheckStuckCondition()
        {
            // Si deberíamos movernos (GO) pero la velocidad es casi nula
            if (rb.linearVelocity.magnitude < 0.5f && _vehicleStatus == Status.GO && !isReacting)
            {
                stuckTimer += Time.deltaTime;
            }
            else
            {
                // Recuperación gradual si nos movemos
                stuckTimer = Mathf.Max(0, stuckTimer - Time.deltaTime * 2);
            }

            // Si llevamos más de 4 segundos parados, consideramos que estamos bloqueados
            if (stuckTimer > 4.0f)
            {
                if (!isStuck)
                {
                    isStuck = true;
                    AttemptReroute(); // ¡INTENTAR CAMBIAR DE RUTA!
                }
            }
            else
            {
                isStuck = false;
            }
        }

        void AttemptReroute()
        {
            // Verificamos si hay opciones alternativas
            if (currentTarget.segment < 0 || currentTarget.segment >= trafficSystem.segments.Count) return;

            Segment currentSeg = trafficSystem.segments[currentTarget.segment];

            // Si es una intersección con múltiples salidas
            if (currentSeg.nextSegments.Count > 1)
            {
                int blockedPathId = futureTarget.segment;

                // Buscar cualquier salida que NO sea la actual bloqueada
                foreach (var seg in currentSeg.nextSegments)
                {
                    if (seg.id != blockedPathId)
                    {
                        // Cambiamos el destino dinámicamente
                        futureTarget.segment = seg.id;
                        futureTarget.waypoint = 0;

                        // Debug visual para saber que ha tomado una decisión
                        // Debug.Log($"Rerouting {this.name}: Bloqueo en {blockedPathId}, cambiando a {seg.id}");

                        // Reseteamos timer para darle oportunidad al nuevo camino y forzamos salida
                        stuckTimer = 0;
                        isStuck = false;
                        return;
                    }
                }
            }
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

            wheelDrive.maxSpeed = initMaxSpeed;

            // 1. CÁLCULO DE DIRECCIÓN
            Transform targetTransform = trafficSystem.segments[currentTarget.segment].waypoints[currentTarget.waypoint].transform;
            Transform futureTargetTransform = trafficSystem.segments[futureTarget.segment].waypoints[futureTarget.waypoint].transform;

            Vector3 desiredVel = targetTransform.position - this.transform.position;
            steering = Mathf.Clamp(this.transform.InverseTransformDirection(desiredVel.normalized).x, -1f, 1f);

            // DETECCIÓN DE GIRO (Visión Túnel)
            bool isTurning = Mathf.Abs(steering) > 0.15f;

            // 2. PARADA (SEMÁFOROS)
            bool shouldStop = (_vehicleStatus == Status.STOP || isReacting);

            if (shouldStop && CheckRightTurnOnRed(targetTransform, futureTargetTransform))
            {
                shouldStop = false;
            }

            if (shouldStop)
            {
                targetAcc = 0;
                targetBrake = 1;
            }
            else
            {
                targetAcc = 1;
                targetBrake = 0;

                if (isTurning)
                {
                    wheelDrive.maxSpeed = Mathf.Min(wheelDrive.maxSpeed, wheelDrive.steeringSpeedMax);
                }

                // 3. SENSORES (LIDAR)
                float hitDist;
                GameObject obstacle = GetDetectedObstacles(isTurning, out hitDist);

                if (obstacle != null)
                {

                    float currentEmergency = emergencyBrakeThresh;
                    float currentSlowDown = slowDownThresh;

                    // Si estamos girando o atascados, permitimos acercarnos más (Agresividad controlada)
                    if (isTurning || isStuck)
                    {
                        currentEmergency = 1.2f;
                        currentSlowDown = 4.0f;
                    }

                    float dangerFactor = 1f - Mathf.Clamp01((hitDist - currentEmergency) / (currentSlowDown - currentEmergency));

                    if (hitDist < currentEmergency)
                    {
                        // Freno total
                        targetAcc = 0;
                        targetBrake = 1;
                        wheelDrive.maxSpeed = 0;
                    }
                    else if (hitDist < currentSlowDown)
                    {
                        // Frenado progresivo
                        targetAcc = Mathf.Lerp(1f, 0f, dangerFactor);
                        targetBrake = Mathf.Lerp(0f, 0.6f, dangerFactor);

                        WheelDrive otherCar = obstacle.GetComponent<WheelDrive>();
                        if (otherCar != null)
                        {
                            float otherSpeed = otherCar.GetSpeedUnit(otherCar.GetComponent<Rigidbody>().linearVelocity.magnitude);
                            // Si estamos girando, ignoramos al otro si va muy lento (para no pararnos en medio del cruce)
                            if (!isTurning || otherSpeed > 5f)
                            {
                                wheelDrive.maxSpeed = Mathf.Min(wheelDrive.maxSpeed, otherSpeed);
                            }
                        }
                    }
                }

                // EMPUJÓN DE SALIDA: Si estamos girando y no hay obstáculo inminente (<1.2m), forzamos gas
                if (isTurning && targetAcc < 0.2f && (hitDist == -1f || hitDist > 1.5f))
                {
                    targetAcc = 0.5f;
                    targetBrake = 0f;
                }
            }

            // 4. FÍSICA SUAVIZADA (Evita frenazos secos)
            currentThrottle = Mathf.MoveTowards(currentThrottle, targetAcc, Time.deltaTime * accelerationSmoothness);
            currentBrake = Mathf.MoveTowards(currentBrake, targetBrake, Time.deltaTime * brakingSmoothness);
            wheelDrive.Move(currentThrottle, steering, currentBrake);
        }

        // --- SISTEMA DE VISIÓN DE TÚNEL ---
        GameObject GetDetectedObstacles(bool isTurning, out float _hitDist)
        {
            GameObject closestObstacle = null;
            float minDist = 1000f;
            _hitDist = -1f;

            // Si giramos -> Visión estrecha (15 grados) para solo ver nuestro carril destino
            // Si recto   -> Visión amplia (50 grados)
            float totalAngle = isTurning ? 15f : 50f;
            float spacing = totalAngle / raysNumber;

            // Si giramos, acortamos la visión (mirar solo cerca)
            float currentLength = isTurning ? raycastLength * 0.7f : raycastLength;

            float initAngle = -(totalAngle / 2f);

            for (int i = 0; i < raysNumber; i++)
            {
                float a = initAngle + (spacing * i);
                Vector3 rayDir = Quaternion.Euler(0, a, 0) * this.transform.forward;
                RaycastHit hit;
                int layerMask = 1 << LayerMask.NameToLayer("AutonomousVehicle");

                // Color Debug: Amarillo=Giro, Rojo=Recta
                Color rayColor = isTurning ? Color.yellow : new Color(1, 0, 0, 0.3f);
                UnityEngine.Debug.DrawRay(raycastAnchor.position, rayDir * currentLength, rayColor);

                if (Physics.Raycast(raycastAnchor.position, rayDir, out hit, currentLength, layerMask))
                {
                    GameObject obj = hit.collider.gameObject;

                    // Filtro de obstáculos
                    if (IsIgnorableObstacle(obj, isTurning)) continue;

                    float dist = Vector3.Distance(this.transform.position, obj.transform.position);
                    if (dist < minDist)
                    {
                        minDist = dist;
                        closestObstacle = obj;
                        if (minDist < 1.0f) break;
                    }
                }
            }
            _hitDist = (minDist == 1000f) ? -1f : minDist;
            return closestObstacle;
        }

        bool IsIgnorableObstacle(GameObject obj, bool amITurning)
        {
            if (obj == this.gameObject) return true;

            float dot = Vector3.Dot(this.transform.forward, obj.transform.forward);
            Rigidbody otherRb = obj.GetComponent<Rigidbody>();
            bool isOtherStopped = (otherRb != null && otherRb.linearVelocity.magnitude < 0.5f);

            // REGLA DE GIRO: Si giro, ignoro a los parados que no estén JUSTO delante
            if (amITurning && isOtherStopped)
            {
                Vector3 dirToObj = (obj.transform.position - this.transform.position).normalized;
                float angleToObj = Vector3.Angle(this.transform.forward, dirToObj);

                // Si no está en un cono de 10 grados frente a mí, LO IGNORO.
                if (angleToObj > 10f) return true;
            }

            // Regla Oncoming (Tráfico en contra)
            if (dot < -0.25f)
            {
                Vector3 localPos = this.transform.InverseTransformPoint(obj.transform.position);
                if (Mathf.Abs(localPos.x) > 1.5f) return true;
            }

            // Regla Perpendicular
            if (Mathf.Abs(dot) < 0.4f && isOtherStopped)
            {
                float dist = Vector3.Distance(this.transform.position, obj.transform.position);
                if (dist > 3.5f) return true;
            }

            return false;
        }

        bool CheckRightTurnOnRed(Transform currentWP, Transform nextWP)
        {
            Vector3 directionToNext = (nextWP.position - currentWP.position).normalized;
            float angle = Vector3.SignedAngle(this.transform.forward, directionToNext, Vector3.up);
            if (angle > 30f)
            {
                if (!IsTrafficIncomingFromLeft(nextWP)) return true;
            }
            return false;
        }

        bool IsTrafficIncomingFromLeft(Transform targetMergePoint)
        {
            Collider[] hits = Physics.OverlapSphere(targetMergePoint.position, 8.0f);
            foreach (var hit in hits)
            {
                if (hit.gameObject != this.gameObject && hit.CompareTag("AutonomousVehicle"))
                {
                    Rigidbody rb = hit.GetComponent<Rigidbody>();
                    if (rb != null && rb.linearVelocity.magnitude > 1.0f) return true;
                }
            }
            return false;
        }

        // --- WAYPOINTS ---
        void WaypointChecker()
        {
            if (trafficSystem == null || currentTarget.segment >= trafficSystem.segments.Count) return;

            GameObject waypoint = trafficSystem.segments[currentTarget.segment].waypoints[currentTarget.waypoint].gameObject;
            Vector3 wpDist = this.transform.InverseTransformPoint(new Vector3(waypoint.transform.position.x, this.transform.position.y, waypoint.transform.position.z));

            if (wpDist.magnitude < waypointThresh)
            {
                currentTarget.waypoint++;
                if (currentTarget.waypoint >= trafficSystem.segments[currentTarget.segment].waypoints.Count)
                {
                    // Actualizamos el segmento pasado antes de cambiar
                    pastTargetSegment = currentTarget.segment;

                    currentTarget.segment = futureTarget.segment;
                    currentTarget.waypoint = 0;
                }
                futureTarget.waypoint = currentTarget.waypoint + 1;
                // Si el futuro waypoint se sale de rango, buscamos el siguiente segmento
                if (futureTarget.segment < trafficSystem.segments.Count &&
                   futureTarget.waypoint >= trafficSystem.segments[futureTarget.segment].waypoints.Count)
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
            // Configuración inicial del futuro objetivo
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
            // Verificación de seguridad
            if (vehicleSegment >= trafficSystem.segments.Count) return 0;

            bool isOnSegment = trafficSystem.segments[vehicleSegment].IsOnSegment(this.transform.position);
            if (!isOnSegment)
            {
                // Si no está en el actual, miramos si sigue en el anterior
                if (pastTargetSegment != -1 && pastTargetSegment < trafficSystem.segments.Count)
                {
                    bool isOnPSegement = trafficSystem.segments[pastTargetSegment].IsOnSegment(this.transform.position);
                    if (isOnPSegement)
                        vehicleSegment = pastTargetSegment;
                }
            }
            return vehicleSegment;
        }
    }
}