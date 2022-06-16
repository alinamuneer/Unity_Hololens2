using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace RosSharp.RosBridgeClient
{
    public class TestshuangSubscriber : UnitySubscriber<MessageTypes.Geometry.PoseStamped>
    {
        //public GameObject cube;
        private MessageTypes.Geometry.PoseStamped message;
        //private MessageTypes.ApriltagRos.AprilTagDetection message_breakdown;
        private bool isMessageReceived;
        public GameObject cube;

        // Start is called before the first frame update
        void Start()
        {
            //message = new MessageTypes.ApriltagRos.AprilTagDetectionArray();
            //message_breakdown = message.detections[0];
            base.Start();

            //ROSConnection.GetOrCreateInstance().Subscribe<RosColor>("tag_detections", TagDetections);
        }

        private void Update()
        {
            if (isMessageReceived)

            // get detections data from the apriltagDetection
            {

                float Position_x = (float)(message.pose.position.x);
                float Position_y = (float)(message.pose.position.y);
                float Position_z = (float)(message.pose.position.z);

                float Orientation_x = (float)(message.pose.orientation.x);
                float Orientation_y = (float)(message.pose.orientation.y);
                float Orientation_z = (float)(message.pose.orientation.z);
                float Orientation_w = (float)(message.pose.orientation.w);

                cube.transform.position = new Vector3(Position_x, Position_y, Position_z);

                cube.transform.rotation = new Quaternion(Orientation_x, Orientation_y, Orientation_z, Orientation_w);

            }
        }
        // Update is called once per frame
        protected override void ReceiveMessage(MessageTypes.Geometry.PoseStamped message)
        {
            this.message = message;
            //this.message_breakdown = message.detections[0];
            isMessageReceived = true;
        }
    }
}
