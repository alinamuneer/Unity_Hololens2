/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.UI;
using System;
using UnityEngine;
using UnityEngine.XR;



namespace RosSharp.RosBridgeClient
{
    public class JogFrameAbsPublisher : UnityPublisher<MessageTypes.Jog.JogFrameAbs>
    {
        public Transform InputTransform;
        public string FrameId;
        public string GroupName;
        public string LinkName;
        public bool AvoidCollisions = true;
        public double DampingFactor = 0.95;
        public event EventHandler ManipulationEnded;
        public bool publish = false; 

        private MessageTypes.Jog.JogFrameAbs message;

        private void Awake()
        {
            InputTransform = new GameObject().transform;
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
            message = new MessageTypes.Jog.JogFrameAbs()
            {
                header = new MessageTypes.Std.Header()
                {
                    frame_id = FrameId
                }
            };
            message.group_name = GroupName;
            message.link_name = LinkName;
            message.avoid_collisions = AvoidCollisions;
            message.damping_factor = DampingFactor;
        }

        private void UpdateMessage()
        {
            message.header.Update();
            message.pose.position = GetGeometryPoint(InputTransform.localPosition.Unity2Ros());
            message.pose.orientation = GetGeometryQuaternion(InputTransform.localRotation.Unity2Ros());
            if (publish)
                Publish(message);
        }
        private MessageTypes.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            MessageTypes.Geometry.Point geometryPoint = new MessageTypes.Geometry.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
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