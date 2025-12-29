using System.Collections.Generic;
using UnityEngine;

namespace TrafficSimulation {
    [System.Serializable]
    public class TrafficSensor {
        public Segment segment; // El carril que vigila este sensor
        public List<GameObject> waitingVehicles = new List<GameObject>();
        
        // Métricas para el algoritmo
        public float totalWaitTime; // T: Suma del tiempo de espera de todos los coches
        public int vehicleCount;    // N: Cantidad de coches

        public TrafficSensor(Segment s) {
            segment = s;
            waitingVehicles = new List<GameObject>();
        }

        public void DetectVehicle(GameObject vehicle) {
            if (!waitingVehicles.Contains(vehicle)) {
                waitingVehicles.Add(vehicle);
                vehicleCount++;
            }
        }

        public void RemoveVehicle(GameObject vehicle) {
            if (waitingVehicles.Contains(vehicle)) {
                waitingVehicles.Remove(vehicle);
                vehicleCount--;
            }
        }

        public void UpdateSensor(float dt) {
            // Aumentamos el tiempo de espera (T) para todos los coches parados
            if (vehicleCount > 0) {
                totalWaitTime += dt * vehicleCount;
            } else {
                totalWaitTime = 0;
            }
        }

        public float GetPriorityWeight(float countCoefficient) {
            // Formula del PDF: P = (N * Coef) + T
            return (vehicleCount * countCoefficient) + totalWaitTime;
        }
    }
}