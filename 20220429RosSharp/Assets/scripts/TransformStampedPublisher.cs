using System;
using UnityEngine;
using UnityEngine.XR;




namespace RosSharp.RosBridgeClient
{
    public class TransformStampedPublisher : UnityPublisher<MessageTypes.Geometry.TransformStamped>
    {
        public Transform PublishedTransform;
        public string parent_frame_ID;
        public string child_frame_ID;
        public bool publish = false;

        private MessageTypes.Geometry.TransformStamped message;

        private void Awake()
        {
            PublishedTransform = new GameObject().transform;
        }

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }


        private void FixedUpdate()
        {
            UpdateMessage();
        }


        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.TransformStamped
            {
                header = new MessageTypes.Std.Header()
                {
                    frame_id = parent_frame_ID
                }
            };
            message.child_frame_id = child_frame_ID;
        }


        private void UpdateMessage()
        {
            message.header.Update();
            message.child_frame_id = child_frame_ID;
            message.transform.translation = GetGeometryVector3(PublishedTransform.localPosition.Unity2Ros());
            message.transform.rotation = GetGeometryQuaternion(PublishedTransform.localRotation.Unity2Ros());
            if (publish)
                Publish(message);
        }

        private MessageTypes.Geometry.Vector3 GetGeometryVector3(Vector3 translation)
        {
            MessageTypes.Geometry.Vector3 geometryVector3 = new MessageTypes.Geometry.Vector3();
            geometryVector3.x = translation.x;
            geometryVector3.y = translation.y;
            geometryVector3.z = translation.z;
            return geometryVector3;
        }

        private MessageTypes.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
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