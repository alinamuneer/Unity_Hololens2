using System;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class EnableTeleoperationPublisher : UnityPublisher<MessageTypes.Std.Int8>
    {
        private MessageTypes.Std.Int8 message;
        public sbyte EnableIndex;
        public bool publish = true;

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
            message = new MessageTypes.Std.Int8();
        }

        private void UpdateMessage()
        {
            message.data = EnableIndex;
            // Debug.Log(message.data);
            if (publish)
            {
                Publish(message);
                publish = false;
            }
        }

    }
}
