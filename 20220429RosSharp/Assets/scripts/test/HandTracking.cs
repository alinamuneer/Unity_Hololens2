using System.Collections;
using System.Collections.Generic;
using Microsoft;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using UnityEngine;
using System;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;


namespace RosSharp.RosBridgeClient
{
    public class HandTracking : MonoBehaviour
    {
        protected PosePublisher PPublisher;
        public GameObject sphereMarker;
        GameObject wristObject;
        //protected GameObject handTransform;
        //protected GameObject goalTransform;

        MixedRealityPose pose;
        MixedRealityPose PoseRight;

        void Start()
        {
            wristObject = Instantiate(sphereMarker, this.transform);

            //handTransform = new GameObject("HandTransform");
            //goalTransform = new GameObject("GoalTransform");
            PPublisher = gameObject.AddComponent<PosePublisher>();
            PPublisher.Topic = "/right/goal_wrist_pose";
            PPublisher.WristPose = new MessageTypes.Geometry.Pose();
            //TSPublisher.parent_frame_ID = "base_link";
            //TSPublisher.child_frame_ID = "rh_palm";
            //PosePublisher.PublishedPose.SetParent(GameObject.Find("base_link").transform); 
            PPublisher.publish = false;
        }

        void Update()
        {
            if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out PoseRight))
                PPublisher.WristPose = MixedRealityPoseToPose(PoseRight);

            wristObject.GetComponent<Renderer>().enabled = false;

            if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out pose))
            {
                wristObject.GetComponent<Renderer>().enabled = true;
                wristObject.transform.position = pose.Position;
                wristObject.transform.rotation = pose.Rotation;

                //handTransform.transform.SetPositionAndRotation(pose.Position, pose.Rotation);
                //handTransform.transform.Rotate(0, -90, -90);
                //goalTransform.transform.localRotation = handTransform.transform.localRotation;

                // Set goal position
                // Vector3 dist = handTransform.transform.position - startHandTransform.transform.position;
                //goalTransform.transform.position = startGoalTransform.transform.position + dist;
                //goalTransform.transform.position = handTransform.transform.position;
                PPublisher.publish = true;
            }
            else
            { PPublisher.publish = false; }

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