using UnityEngine;


namespace RosSharp.RosBridgeClient
{
    public class EnableTeleoperationSubscriber : UnitySubscriber<MessageTypes.Std.Int8>
    {
        private bool isMessageReceived;
        private MessageTypes.Std.Int8 message;

        protected override void Start()
        {
            base.Start();
        }

        private void Update()
        {
            if (isMessageReceived)
            {
                isMessageReceived = false;
                if (message.data == 0)
                {
                    GameObject.Find("RosConnector").GetComponent<PR2LeftArmTeleop>().StopTeleop();
                    GameObject.Find("RosConnector").GetComponent<PR2RightArmTeleop>().StopTeleop();
                    GameObject.Find("RosConnector").GetComponent<PR2GripperTeleop>().StopTeleop();
                    // GameObject.Find("RosConnector").GetComponent<RightHandTeleop>().StopTeleop();
                    GameObject.Find("RosConnector").GetComponent<SHParallelGraspTeleop>().StopTeleop();
                }
                else if (message.data == 1)
                {
                    GameObject.Find("RosConnector").GetComponent<PR2LeftArmTeleop>().StartTeleop();
                    GameObject.Find("RosConnector").GetComponent<PR2GripperTeleop>().StartTeleop();
                }
                else if (message.data == 2)
                {
                    GameObject.Find("RosConnector").GetComponent<PR2RightArmTeleop>().StartTeleop();
                    // GameObject.Find("RosConnector").GetComponent<RightHandTeleop>().StartTeleop();
                    GameObject.Find("RosConnector").GetComponent<SHParallelGraspTeleop>().StartTeleop();
                }
                else if (message.data == 3)
                {
                    GameObject.Find("RosConnector").GetComponent<PR2LeftArmTeleop>().StopTeleop();
                    GameObject.Find("RosConnector").GetComponent<PR2GripperTeleop>().StopTeleop();
                }
                else if (message.data == 4)
                {
                    GameObject.Find("RosConnector").GetComponent<PR2RightArmTeleop>().StopTeleop();
                    //GameObject.Find("RosConnector").GetComponent<RightHandTeleop>().StopTeleop();
                    GameObject.Find("RosConnector").GetComponent<SHParallelGraspTeleop>().StopTeleop();
                }
            }
        }

        protected override void ReceiveMessage(MessageTypes.Std.Int8 message)
        {
            this.message = message;
            isMessageReceived = true;
        }
    }
}