using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using UnityEngine;
using System;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;


namespace RosSharp.RosBridgeClient
{
    public class HandPublisher : MonoBehaviour
    {
        public string TopicRight;
        //public string TopicLeft;

        private PoseArrayPublisher rightHandPublisher;
        //private PoseArrayPublisher leftHandPublisher;

        private static readonly int jointCount = Enum.GetNames(typeof(TrackedHandJoint)).Length;
        public Handedness recordingHandRight = Handedness.Right;
        //public Handedness recordingHandLeft = Handedness.Left;

        private void Start()
        {
            // Right hand
            rightHandPublisher = gameObject.AddComponent<PoseArrayPublisher>();
            rightHandPublisher.Topic = TopicRight;
            rightHandPublisher.FrameId = "base_link";
            rightHandPublisher.Poses = new MessageTypes.Geometry.Pose[jointCount];

            // Left hand
            //leftHandPublisher = gameObject.AddComponent<PoseArrayPublisher>();
            //leftHandPublisher.Topic = TopicLeft;
            //leftHandPublisher.FrameId = "base_link";
            //leftHandPublisher.Poses = new MessageTypes.Geometry.Pose[jointCount];
        }

        private void FixedUpdate()
        {
            MixedRealityPose[] jointPosesRight = new MixedRealityPose[jointCount];
            //MixedRealityPose[] jointPosesLeft = new MixedRealityPose[jointCount];

            for (int i = 0; i < jointCount; ++i)
            {
                HandJointUtils.TryGetJointPose((TrackedHandJoint)i, recordingHandRight, out jointPosesRight[i]);
                rightHandPublisher.Poses[i] = MixedRealityPoseToPose(jointPosesRight[i]);

                // HandJointUtils.TryGetJointPose((TrackedHandJoint)i, recordingHandLeft, out jointPosesLeft[i]);
                // leftHandPublisher.Poses[i] = MixedRealityPoseToPose(jointPosesLeft[i]);
            }
        }

        private MessageTypes.Geometry.Pose MixedRealityPoseToPose(MixedRealityPose pose)
        {
            return new MessageTypes.Geometry.Pose(
                    GetGeometryPoint(pose.Position),
                    GetGeometryQuaternion(pose.Rotation));
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