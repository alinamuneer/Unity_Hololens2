/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */



using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient.MessageTypes.Pr2GripperSensor
{
    public class PR2GripperSlipServoData : Message
    {
        public const string RosMessageName = "pr2_gripper_sensor_msgs/PR2GripperSlipServoData";

        //  time the data was recorded at
        public Time stamp { get; set; }
        //  the amount of deformation from action start (in meters)
        public double deformation { get; set; }
        //  the force experinced by the finger Pads  (N)
        //  NOTE:this ignores data from the edges of the finger pressure
        public double left_fingertip_pad_force { get; set; }
        public double right_fingertip_pad_force { get; set; }
        //  the current virtual parallel joint effort of the gripper (in N)
        public double joint_effort { get; set; }
        //  true if the object recently slipped
        public bool slip_detected { get; set; }
        //  true if we are at or exceeding the deformation limit
        //  (see wiki page and param server for more info)
        public bool deformation_limit_reached { get; set; }
        //  true if we are at or exceeding our force 
        //  (see wiki page and param server for more info)
        public bool fingertip_force_limit_reached { get; set; }
        //  true if the controller thinks the gripper is empty
        //  (see wiki page for more info)
        public bool gripper_empty { get; set; }
        //  the control state of our realtime controller
        public PR2GripperSensorRTState rtstate { get; set; }

        public PR2GripperSlipServoData()
        {
            this.stamp = new Time();
            this.deformation = 0.0;
            this.left_fingertip_pad_force = 0.0;
            this.right_fingertip_pad_force = 0.0;
            this.joint_effort = 0.0;
            this.slip_detected = false;
            this.deformation_limit_reached = false;
            this.fingertip_force_limit_reached = false;
            this.gripper_empty = false;
            this.rtstate = new PR2GripperSensorRTState();
        }

        public PR2GripperSlipServoData(Time stamp, double deformation, double left_fingertip_pad_force, double right_fingertip_pad_force, double joint_effort, bool slip_detected, bool deformation_limit_reached, bool fingertip_force_limit_reached, bool gripper_empty, PR2GripperSensorRTState rtstate)
        {
            this.stamp = stamp;
            this.deformation = deformation;
            this.left_fingertip_pad_force = left_fingertip_pad_force;
            this.right_fingertip_pad_force = right_fingertip_pad_force;
            this.joint_effort = joint_effort;
            this.slip_detected = slip_detected;
            this.deformation_limit_reached = deformation_limit_reached;
            this.fingertip_force_limit_reached = fingertip_force_limit_reached;
            this.gripper_empty = gripper_empty;
            this.rtstate = rtstate;
        }
    }
}