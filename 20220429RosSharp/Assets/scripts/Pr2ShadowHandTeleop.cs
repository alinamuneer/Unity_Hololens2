using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

namespace RosSharp.RosBridgeClient
{
    public class Pr2ShadowHandTeleop : MonoBehaviour
    {
        protected HandPositionPublisher HPPublisher;
        protected bool enabledTeleop = false;
        private MessageTypes.Geometry.Pose goalPose;
        private Transform PublishedTransform;

        private List<MixedRealityPose> rightHandKeypoints;

        private List<GameObject> keypointTransform;
        private List<GameObject> startKeypointTransform;
        private List<GameObject> keypointGoalTransform;
        private List<GameObject> startKeypointGoalTransform;
        private List<GameObject> currentEndEffectorTransform;


        private MixedRealityPose thumbTip;
        private MixedRealityPose thumbProximal;
        private MixedRealityPose thumbMeta;
        private MixedRealityPose indexTip;
        private MixedRealityPose indexMiddle;
        private MixedRealityPose indexKnuckle;
        private MixedRealityPose middleTip;
        private MixedRealityPose middleKnuckle;
        private MixedRealityPose middleMiddle;
        private MixedRealityPose ringTip;
        private MixedRealityPose ringMiddle;
        private MixedRealityPose ringKnuckle;

        // Start is called before the first frame update
        void Start()
        {
            HPPublisher = gameObject.AddComponent<HandPositionPublisher>();
            HPPublisher.Topic = "/hand/goal_position";
            HPPublisher.positionData = new List<MessageTypes.Geometry.Pose>();
            goalPose = new MessageTypes.Geometry.Pose();
            PublishedTransform = new GameObject().transform;
            PublishedTransform.SetParent(GameObject.Find("rh_palm/rh_wrist").transform);

            rightHandKeypoints = new List<MixedRealityPose>();

            currentEndEffectorTransform.Add(GameObject.Find("rh_thtip"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_thproximal"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_thmiddle"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_fftip"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_ffproximal"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_ffmiddle"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_mftip"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_mfproximal"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_mfmiddle"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_rftip"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_rfproximal"));
            currentEndEffectorTransform.Add(GameObject.Find("rh_rfmiddle"));

            keypointTransform.Add(new GameObject("thumbTipTransform"));
            keypointTransform.Add(new GameObject("thumbMetacarpalsTransform"));
            keypointTransform.Add(new GameObject("thumbProximalTransform"));
            keypointTransform.Add(new GameObject("indexTipTransform"));
            keypointTransform.Add(new GameObject("indexKnuckleTransform"));
            keypointTransform.Add(new GameObject("indexMiddleTransform"));
            keypointTransform.Add(new GameObject("middleTipTransform"));
            keypointTransform.Add(new GameObject("middleKnuckleTransform"));
            keypointTransform.Add(new GameObject("middleMiddleTransform"));
            keypointTransform.Add(new GameObject("ringTipTransform"));
            keypointTransform.Add(new GameObject("ringKnuckleTransform"));
            keypointTransform.Add(new GameObject("ringMiddleTransform"));

            startKeypointTransform.Add(new GameObject("startThumbTipTransform"));
            startKeypointTransform.Add(new GameObject("startThumbMetacarpalsTransform"));
            startKeypointTransform.Add(new GameObject("startThumbProximalTransform"));
            startKeypointTransform.Add(new GameObject("startIndexTipTransform"));
            startKeypointTransform.Add(new GameObject("startIndexKnuckleTransform"));
            startKeypointTransform.Add(new GameObject("startIndexMiddleTransform"));
            startKeypointTransform.Add(new GameObject("startMiddleTipTransform"));
            startKeypointTransform.Add(new GameObject("startMiddleKnuckleTransform"));
            startKeypointTransform.Add(new GameObject("startMiddleMiddleTransform"));
            startKeypointTransform.Add(new GameObject("startRingTipTransform"));
            startKeypointTransform.Add(new GameObject("startRingKnuckleTransform"));
            startKeypointTransform.Add(new GameObject("startRingMiddleTransform"));

            keypointGoalTransform.Add(new GameObject("thumbTipGoalTransform"));
            keypointGoalTransform.Add(new GameObject("thumbMetacarpalsGoalTransform"));
            keypointGoalTransform.Add(new GameObject("thumbProximalGoalTransform"));
            keypointGoalTransform.Add(new GameObject("indexTipGoalTransform"));
            keypointGoalTransform.Add(new GameObject("indexKnuckleGoalTransform"));
            keypointGoalTransform.Add(new GameObject("indexMiddleGoalTransform"));
            keypointGoalTransform.Add(new GameObject("middleTipGoalTransform"));
            keypointGoalTransform.Add(new GameObject("middleKnuckleGoalTransform"));
            keypointGoalTransform.Add(new GameObject("middleMiddleGoalTransform"));
            keypointGoalTransform.Add(new GameObject("ringTipGoalTransform"));
            keypointGoalTransform.Add(new GameObject("ringKnuckleGoalTransform"));
            keypointGoalTransform.Add(new GameObject("ringMiddleGoalTransform"));

            startKeypointGoalTransform.Add(new GameObject("startThumbTipGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startThumbMetacarpalsGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startThumbProximalGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startIndexTipGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startIndexKnuckleGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startIndexMiddleGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startMiddleTipGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startMiddlKnuckleGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startMiddleMiddleGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startRingTipGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startRingKnuckleGoalTransform"));
            startKeypointGoalTransform.Add(new GameObject("startRingMiddleGoalTransform"));

            for (int i = 0; i < keypointTransform.Count; i++)
            {
                keypointTransform[i].transform.SetParent(startKeypointTransform[i].transform);
                keypointGoalTransform[i].transform.SetParent(startKeypointGoalTransform[i].transform);
            }
        }

        void Update()
        {
            if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Right, out thumbTip) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbProximalJoint, Handedness.Right, out thumbProximal) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbMetacarpalJoint, Handedness.Right, out thumbMeta) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, Handedness.Right, out indexTip) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexMiddleJoint, Handedness.Right, out indexMiddle) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexMetacarpal, Handedness.Right, out indexKnuckle) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.MiddleTip, Handedness.Right, out middleTip) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.MiddleKnuckle, Handedness.Right, out middleKnuckle) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.MiddleMiddleJoint, Handedness.Right, out middleMiddle) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.RingTip, Handedness.Right, out ringTip) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.RingMiddleJoint, Handedness.Right, out ringMiddle) &&
                HandJointUtils.TryGetJointPose(TrackedHandJoint.RingKnuckle, Handedness.Right, out ringKnuckle))
            {
                if (!enabledTeleop)
                    return;
                else
                    HPPublisher.publish = true;

                rightHandKeypoints.Add(thumbTip);
                rightHandKeypoints.Add(thumbMeta);
                rightHandKeypoints.Add(thumbProximal);

                rightHandKeypoints.Add(indexTip);
                rightHandKeypoints.Add(indexKnuckle);
                rightHandKeypoints.Add(indexMiddle);

                rightHandKeypoints.Add(middleTip);
                rightHandKeypoints.Add(middleKnuckle);
                rightHandKeypoints.Add(middleMiddle);

                rightHandKeypoints.Add(ringTip);
                rightHandKeypoints.Add(indexKnuckle);
                rightHandKeypoints.Add(ringMiddle);

                for (int i = 0; i < rightHandKeypoints.Count; i++)
                {
                    keypointTransform[i].transform.SetPositionAndRotation(rightHandKeypoints[i].Position, rightHandKeypoints[i].Rotation);
                    // Set goal position
                    Vector3 dist = keypointTransform[i].transform.position - startKeypointTransform[i].transform.position;
                    keypointGoalTransform[i].transform.position = startKeypointGoalTransform[i].transform.position + dist;

                    // Update hand position data
                    PublishedTransform.SetPositionAndRotation(keypointGoalTransform[i].transform.position, keypointGoalTransform[i].transform.rotation);
                    goalPose.position = GetGeometryPoint(PublishedTransform.transform.localPosition.Unity2Ros());
                    HPPublisher.positionData.Add(goalPose);
                }
            }
            else
                 HPPublisher.publish = false; 
        }

        private MessageTypes.Geometry.Point GetGeometryPoint(Vector3 translation)
        {
            MessageTypes.Geometry.Point geometryPoint = new MessageTypes.Geometry.Point();
            geometryPoint.x = translation.x;
            geometryPoint.y = translation.y;
            geometryPoint.z = translation.z;
            return geometryPoint;
        }

        public void StartTeleop()
        {
            enabledTeleop = true;

            for (int i = 0; i < keypointTransform.Count; i++)
            {
                // get current keypoint pose 
                startKeypointTransform[i].transform.SetPositionAndRotation(rightHandKeypoints[i].Position, rightHandKeypoints[i].Rotation);

                // get current position of the keypoints goal
                startKeypointGoalTransform[i].transform.SetPositionAndRotation(currentEndEffectorTransform[i].transform.position, currentEndEffectorTransform[i].transform.rotation);
            }
        }

        public void StopTeleop()
        {
            enabledTeleop = false;
        }
    }
}
