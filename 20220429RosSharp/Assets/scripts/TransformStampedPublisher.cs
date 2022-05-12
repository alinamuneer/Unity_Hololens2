using System;
using UnityEngine;
using UnityEngine.XR;




namespace RosSharp.RosBridgeClient
{
    public class TransformStampedPublisher : UnityPublisher<MessageTypes.Geometry.TransformStamped>
    {
        public Transform PublishedTransform;
        public string child_frame_id;
        


        private MessageTypes.Geometry.TransformStamped message;

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
                    frame_id = child_frame_id
                }
            };
        }

        private void UpdateMessage()
        {
            message.header.Update();
            GetGeometryVector3(PublishedTransform.position.Unity2Ros(), message.transform.translation);
            GetGeometryQuaternion(PublishedTransform.rotation.Unity2Ros(), message.transform.rotation);

            Publish(message);
        }

        private static void GetGeometryVector3(Vector3 translation, MessageTypes.Geometry.Vector3 geometryVector3)
        {
            geometryVector3.x = translation.x;
            geometryVector3.y = translation.y;
            geometryVector3.z = translation.z;
        }

        private static void GetGeometryQuaternion(Quaternion quaternion, MessageTypes.Geometry.Quaternion geometryQuaternion)
        {
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
        }
    }
}
