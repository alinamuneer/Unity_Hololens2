using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

namespace RosSharp.RosBridgeClient
{
    public class HandPositionPublisher : UnityPublisher<MessageTypes.Geometry.PoseArray>
    {

        private MessageTypes.Geometry.PoseArray message;
        public bool publish = false;
        public List<MessageTypes.Geometry.Pose> positionData;

        // Start is called before the first frame update
        void Start()
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
            message = new MessageTypes.Geometry.PoseArray
            {
                header = new MessageTypes.Std.Header(),
                poses = new MessageTypes.Geometry.Pose[12]
            };
            positionData = new List<MessageTypes.Geometry.Pose>();
        }

        private void UpdateMessage()
        {
            message.header.Update();
            for (int i = 0; i < positionData.Count; i++)
            {
                message.poses[i] = positionData[i];
            }
            if (publish)
                Publish(message);
        }
    }
}
