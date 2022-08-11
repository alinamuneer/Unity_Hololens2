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
    public class PR2GripperFindContactDataSubscriber : UnitySubscriber<MessageTypes.Pr2GripperSensor.PR2GripperFindContactData>
    {
        public MessageTypes.Pr2GripperSensor.PR2GripperFindContactData message;

        protected override void Start()
        {
            base.Start();
            message = new MessageTypes.Pr2GripperSensor.PR2GripperFindContactData();
        }

        protected override void ReceiveMessage(MessageTypes.Pr2GripperSensor.PR2GripperFindContactData message)
        {
            this.message = message;
        }
    }
}