using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using System;


namespace RosSharp.RosBridgeClient
{
    public class RightHandTeleop : MonoBehaviour
    {
        public string TopicRight;
        protected bool enabledTeleop = false;

        private PoseArrayPublisher rightHandPublisher;

        private static readonly int jointCount = Enum.GetNames(typeof(TrackedHandJoint)).Length;
        public Handedness recordingHandRight = Handedness.Right;

        GameObject[] keypointsTransform;
        string[] keypointsTransformNames;

        void Start()
        {
            // Right hand
            rightHandPublisher = gameObject.AddComponent<PoseArrayPublisher>();
            rightHandPublisher.Topic = TopicRight;
            rightHandPublisher.FrameId = "base_footprint";
            rightHandPublisher.Poses = new MessageTypes.Geometry.Pose[jointCount];

            keypointsTransformNames = new string[27]{
                "empltyTransform",
                "wristTransform",
                "palmTransform",
                "thumbMetacarpalsTransform","thumbProximalTransform", "thumbDistalTransform", "thumbTipTransform",
                "indexMetacarpalTransform","indexKnuckleTransform", "indexMiddleTransform", "indexDistalTransform", "indexTipTransform",
                "middleMetacarpalTransform","middleKnuckleTransform", "middleMiddleTransform", "middleDistalTransform", "middleTipTransform",
                "ringMetacarpalTransform","ringKnuckleTransform", "ringMiddleTransform", "ringDistalTransform", "ringTipTransform",
                "pinkyMetacarpalTransform","pinkyKnuckleTransform", "pinkeyMiddleTransform", "pinkyDistalTransform", "pinkyTipTransform",
            };

            // initialize keypointsTransform 
            keypointsTransform = new GameObject[jointCount];

            for (int i = 0; i < jointCount; ++i)
            {
                GameObject keypointTransform = new GameObject(keypointsTransformNames[i]);
                keypointTransform.transform.SetParent(GameObject.Find("base_footprint").transform);
                keypointsTransform[i] = keypointTransform;
            }
        }

        // Update is called once per frame
        private void Update()
        {
            if (!enabledTeleop)
                return;

            MixedRealityPose[] jointPosesRight = new MixedRealityPose[jointCount];

            for (int i = 0; i < jointCount; ++i)
            {
                HandJointUtils.TryGetJointPose((TrackedHandJoint)i, recordingHandRight, out jointPosesRight[i]);
                keypointsTransform[i].transform.SetPositionAndRotation(jointPosesRight[i].Position, jointPosesRight[i].Rotation);
                keypointsTransform[i].transform.Rotate(0, -90, -90);

                MessageTypes.Geometry.Pose pose = new MessageTypes.Geometry.Pose();
                pose.position = GetGeometryPoint(keypointsTransform[i].transform.position.Unity2Ros());
                pose.orientation = GetGeometryQuaternion(keypointsTransform[i].transform.rotation.Unity2Ros());

                Debug.LogError(pose.position.x);

                rightHandPublisher.Poses[i] = pose;
            }
        }


        private MessageTypes.Geometry.Point GetGeometryPoint(UnityEngine.Vector3 position)
        {
            MessageTypes.Geometry.Point geometryPoint = new MessageTypes.Geometry.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }

        private MessageTypes.Geometry.Quaternion GetGeometryQuaternion(UnityEngine.Quaternion quaternion)
        {
            MessageTypes.Geometry.Quaternion geometryQuaternion = new MessageTypes.Geometry.Quaternion();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }


        public void StartTeleop()
        {
            enabledTeleop = true;
            rightHandPublisher.publish = true;
        }

        public void StopTeleop()
        {
            enabledTeleop = false;
            rightHandPublisher.publish = false;
        }
    }
}
