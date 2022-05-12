using UnityEngine;
using Microsoft.MixedReality.Toolkit.Utilities;
using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public abstract class ArmTeleop : MonoBehaviour
    {
        // Jog Frame stuff
        protected TransformStampedPublisher TSPublisher;
        protected bool enabledTeleop = false;
        protected GameObject handTransform;
        protected GameObject goalTransform;
        protected GameObject startHandTransform;
        protected GameObject startGoalTransform;
        protected GameObject currentEndEffectorTransform;

        // Visualization on Hand
        protected GameObject gripperViz;

        // Goal Gripper visualization
        protected GameObject gripperVizGoal;
        protected GameObject goalBall;

        // Hand detection stuff
        public Handedness recordingHand;

        private Material red;
        private Material green;

        protected virtual void Start()
        {
            goalTransform = new GameObject("GoalTransform");
            startGoalTransform = new GameObject("StartGoalTransform");
            handTransform = new GameObject("HandTransform");
            startHandTransform = new GameObject("StartHandTransform");

            handTransform.transform.SetParent(startHandTransform.transform);
            goalTransform.transform.SetParent(startGoalTransform.transform);

            red = Resources.Load("Materials/MRTK_Standard_TransparentRed", typeof(Material)) as Material;
            green = Resources.Load("Materials/MRTK_Standard_TransparentGreen", typeof(Material)) as Material;
        }

        protected abstract void Update();

        protected abstract void UpdateHandTransform();

        protected abstract void UpdateGripperPose();

        protected abstract void UpdateGoalPose();


        protected void UpdateGoalTransform()
        {
            if (!enabledTeleop)
                return;

            // Set goal rotation
            goalTransform.transform.localRotation = handTransform.transform.localRotation;

            // Set goal position
            Vector3 dist = handTransform.transform.position - startHandTransform.transform.position;
            goalTransform.transform.position = startGoalTransform.transform.position + dist;

        }

        protected void SetJogGoal()
        {
            if (!enabledTeleop)
                return;

            TSPublisher.PublishedTransform.SetPositionAndRotation(goalTransform.transform.position, goalTransform.transform.rotation);
        }

        public void StartTeleop()
        {

            // get current position of hand 
            startHandTransform.transform.SetPositionAndRotation(handTransform.transform.position, handTransform.transform.rotation);
            // get current position of goal 
            startGoalTransform.transform.SetPositionAndRotation(currentEndEffectorTransform.transform.position, currentEndEffectorTransform.transform.rotation);

            goalBall.GetComponent<MeshRenderer>().material = green;
            enabledTeleop = true;
            TSPublisher.publish = true;
        }

        public void StopTeleop()
        {
            goalBall.GetComponent<MeshRenderer>().material = red;
            enabledTeleop = false;
            TSPublisher.publish = false;
        }

        // assuming qArray.Length > 1
        protected Quaternion AverageQuaternion(List<Quaternion> qArray)
        {
            Quaternion qAvg = qArray[0];
            float weight;
            for (int i = 1; i < qArray.Count; i++)
            {
                weight = 1.0f / (float)(i + 1);
                qAvg = Quaternion.Slerp(qAvg, qArray[i], weight);
            }
            return qAvg;
        }

        public static Vector3 GetNormalVectorOnLine(Vector3 linePnt, Vector3 lineDir, Vector3 pnt)
        {
            lineDir.Normalize();//this needs to be a unit vector
            var v = pnt - linePnt;
            var d = Vector3.Dot(v, lineDir);
            var closestPoint = linePnt + lineDir * d;
            return pnt - closestPoint;
        }

        protected void TransformParentWithChild(Transform parent, Transform child, Vector3 position, Quaternion rotation)
        {
            parent.rotation = rotation * Quaternion.Inverse(child.localRotation);
            parent.position = position + (parent.position - child.position);
        }
    }
}