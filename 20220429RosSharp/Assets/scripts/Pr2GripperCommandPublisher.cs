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

// Added allocation free alternatives
// UoK , 2019, Odysseas Doumas (od79@kent.ac.uk / odydoum@gmail.com)

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class Pr2GripperCommandPublisher : UnityPublisher<MessageTypes.Pr2Controllers.Pr2GripperCommand>
    {
        public float Position;
        public float MaxEffort;

        public bool publish = false;

        private MessageTypes.Pr2Controllers.Pr2GripperCommand message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            if (publish)
                UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Pr2Controllers.Pr2GripperCommand();
        }

        private void UpdateMessage()
        {
            message.position = Position;
            message.max_effort = MaxEffort;
            Publish(message);
        }
    }
}