using System;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class PosePublisher : UnityPublisher<MessageTypes.Geometry.Pose>
    {
        public MessageTypes.Geometry.Pose WristPose;
        public bool publish = false;
        private MessageTypes.Geometry.Pose message;

       
        protected override void Start()
        {
            base.Start();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void UpdateMessage()
        {
            message = WristPose;
            if (publish)
                Publish(message);
        }

    }
}
