using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using UnityEngine;
using System;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.XR;

namespace RosSharp.RosBridgeClient
{
    public class RightHandPublisherWRTFrame : MonoBehaviour
    {
        public string TopicRight;

        private PoseArrayPublisher rightHandPublisher;

        private static readonly int jointCount = Enum.GetNames(typeof(TrackedHandJoint)).Length;
        public Handedness recordingHandRight = Handedness.Right;

        private List<GameObject> keypointTransform;
        private List<string> keypointTransformNames; 

        private void Start()
        {
            // Right hand
            rightHandPublisher = gameObject.AddComponent<PoseArrayPublisher>();
            rightHandPublisher.Topic = TopicRight;
            rightHandPublisher.FrameId = "base_link";
            rightHandPublisher.Poses = new MessageTypes.Geometry.Pose[jointCount];

            keypointTransformNames = new List<string>
            {
                "empltyTransform",
                "wristTransform",
                "palmTransform",
                "thumbMetacarpalsTransform","thumbProximalTransform", "thumbDistalTransform", "thumbTipTransform",
                "indexMetacarpalTransform","indexKnuckleTransform", "indexMiddleTransform", "indexDistalTransform", "indexTipTransform",
                "middleMetacarpalTransform","middleKnuckleTransform", "middleMiddleTransform", "middleDistalTransform", "middleTipTransform",
                "ringMetacarpalTransform","ringKnuckleTransform", "ringMiddleTransform", "ringDistalTransform", "ringTipTransform",
                "pinkyMetacarpalTransform","pinkyKnuckleTransform", "pinkeyMiddleTransform", "pinkyDistalTransform", "pinkyTipTransform",
            };

            for (int i = 0; i < jointCount; ++i)
            {
                keypointTransform.Add(new GameObject(keypointTransformNames[i]));
                keypointTransform[i].transform.SetParent(GameObject.Find("base_link").transform);
            }
    }

        private void FixedUpdate()
        {
            MixedRealityPose[] jointPosesRight = new MixedRealityPose[jointCount];

            for (int i = 0; i < jointCount; ++i)
            {
                HandJointUtils.TryGetJointPose((TrackedHandJoint)i, recordingHandRight, out jointPosesRight[i]);

                keypointTransform[i].transform.SetPositionAndRotation(jointPosesRight[i].Position, jointPosesRight[i].Rotation);
                keypointTransform[i].transform.Rotate(0, -90, -90);

                rightHandPublisher.Poses[i].position = GetGeometryPoint(keypointTransform[i].transform.localPosition.Unity2Ros());
                rightHandPublisher.Poses[i].orientation = GetGeometryQuaternion(keypointTransform[i].transform.localRotation.Unity2Ros());
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
    }
}