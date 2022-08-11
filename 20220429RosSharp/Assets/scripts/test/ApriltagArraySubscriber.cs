using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace RosSharp.RosBridgeClient
{
    public class ApriltagArraySubscriber : UnitySubscriber<MessageTypes.ApriltagRos.AprilTagDetectionArray>
    {
       private MessageTypes.ApriltagRos.AprilTagDetectionArray message;
       private bool isMessageReceived;
       public GameObject cube;
        
        // Start is called before the first frame update
        void Start()
        {
            //message = new MessageTypes.ApriltagRos.AprilTagDetectionArray();
            base.Start();
        }

        private void Update()
        {
            if (isMessageReceived)

            // get detections data from the apriltagDetection
            {
                
                float Position_x = (float)(message.detections[0].pose.pose.pose.position.x);
                float Position_y = (float)(message.detections[0].pose.pose.pose.position.y);
                float Position_z = (float)(message.detections[0].pose.pose.pose.position.z);

                float Orientation_x = (float)(message.detections[0].pose.pose.pose.orientation.x);
                float Orientation_y = (float)(message.detections[0].pose.pose.pose.orientation.y);
                float Orientation_z = (float)(message.detections[0].pose.pose.pose.orientation.z);
                float Orientation_w = (float)(message.detections[0].pose.pose.pose.orientation.w);

                cube.transform.position = new Vector3(Position_x, Position_y, Position_z);

                cube.transform.rotation = new Quaternion(Orientation_x, Orientation_y, Orientation_z, Orientation_w);

            }
        }
        // Update is called once per frame
        protected override void ReceiveMessage(MessageTypes.ApriltagRos.AprilTagDetectionArray message)
        {
            this.message = message;
            //this.message_breakdown = message.detections[0];
            isMessageReceived = true;
        }
    }
}
