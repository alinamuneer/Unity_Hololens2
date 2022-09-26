/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */



namespace RosSharp.RosBridgeClient.MessageTypes.Pr2GripperSensor
{
    public class PR2GripperSensorRTState : Message
    {
        //public const string RosMessageName = "pr2_gripper_sensor_msgs/PR2GripperSensorRTState";
        public override string RosMessageName => "pr2_gripper_sensor_msgs/PR2GripperSensorRTState";

        //  the control state of our realtime controller
        public sbyte realtime_controller_state { get; set; }
        //  predefined values to indicate our realtime_controller_state
        public const sbyte DISABLED = 0;
        public const sbyte POSITION_SERVO = 3;
        public const sbyte FORCE_SERVO = 4;
        public const sbyte FIND_CONTACT = 5;
        public const sbyte SLIP_SERVO = 6;

        public PR2GripperSensorRTState()
        {
            this.realtime_controller_state = 0;
        }

        public PR2GripperSensorRTState(sbyte realtime_controller_state)
        {
            this.realtime_controller_state = realtime_controller_state;
        }
    }
}
