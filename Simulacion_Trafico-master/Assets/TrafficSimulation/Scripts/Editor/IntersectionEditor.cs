
using UnityEngine;
using UnityEditor;

namespace TrafficSimulation {
    [CustomEditor(typeof(Intersection))]
    public class IntersectionEditor : Editor
    {
        private Intersection intersection;

        void OnEnable(){
            intersection = target as Intersection;
        }

        public override void OnInspectorGUI(){
            //Tipo de interseccion
            intersection.intersectionType = (IntersectionType) EditorGUILayout.EnumPopup("Intersection type", intersection.intersectionType);
            EditorGUILayout.Space();

            //Sección STOP
            EditorGUI.BeginDisabledGroup(intersection.intersectionType != IntersectionType.STOP);
            EditorGUILayout.LabelField("Configuración Stop", EditorStyles.boldLabel);
            SerializedProperty sPrioritySegments = serializedObject.FindProperty("prioritySegments");
            EditorGUILayout.PropertyField(sPrioritySegments, new GUIContent("Priority Segments"), true);
            serializedObject.ApplyModifiedProperties();
            EditorGUI.EndDisabledGroup();

            EditorGUILayout.Space();

            //Seccion Semaforo Inteligente

            EditorGUI.BeginDisabledGroup(intersection.intersectionType != IntersectionType.TRAFFIC_LIGHT);

            EditorGUILayout.LabelField("Semaforo Inteligente (AI)", EditorStyles.boldLabel);

            //Nuevas variables de la IA
            intersection.minGreenTime = EditorGUILayout.FloatField("Min Green Time (s)", intersection.minGreenTime);
            intersection.maxWaitTimeSafety = EditorGUILayout.FloatField("Max Wait Safety (s)", intersection.maxWaitTimeSafety);
            intersection.urgencyCoefficient = EditorGUILayout.FloatField("Urgency Coeff (N vs T)", intersection.urgencyCoefficient);
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Grupos de Semáforos", EditorStyles.boldLabel);
            SerializedProperty sLightsNbr1 = serializedObject.FindProperty("lightsNbr1");
            SerializedProperty sLightsNbr2 = serializedObject.FindProperty("lightsNbr2");
            EditorGUILayout.PropertyField(sLightsNbr1, new GUIContent("Grupo 1 (Norte/Sur)"), true);
            EditorGUILayout.PropertyField(sLightsNbr2, new GUIContent("Grupo 2 (Este/Oeste)"), true);
            serializedObject.ApplyModifiedProperties();
            EditorGUI.EndDisabledGroup();

        }
    }
}
