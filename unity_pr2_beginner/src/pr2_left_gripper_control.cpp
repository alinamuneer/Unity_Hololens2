#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperForceServoAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperForceServoAction> ForceClient;


class Gripper{
private:
  GripperClient* gripper_client_;

  ros::Subscriber sub_;
  ros::Subscriber sub_contact_;
  ros::Subscriber sub_position_;
  ros::Publisher pub_;

  bool in_contact = false;
  float ref_force = 0;
  bool first_contact = true;

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("/l_gripper_controller/gripper_action", true);

    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the l_gripper_sensor_controller/gripper_action action server to come up");
    }

    ros::NodeHandle nh;
    sub_position_ = nh.subscribe("/hololens_gripper_control/position_command", 1, &Gripper::positionCallback, this);
    sub_contact_ = nh.subscribe("/l_gripper_sensor_controller/contact_state", 1, &Gripper::contactCallback, this);
    pub_ = nh.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/l_gripper_controller/command", 1);

  }

  ~Gripper(){
    delete gripper_client_;
  }

  void positionCallback(const pr2_controllers_msgs::Pr2GripperCommand::ConstPtr &msg){
    moveToPosition(msg->position);
  }

  void contactCallback(const pr2_gripper_sensor_msgs::PR2GripperFindContactData::ConstPtr &msg){
    if (msg->left_fingertip_pad_contact && msg->right_fingertip_pad_contact)
    {
      gripper_client_->cancelGoal();
      in_contact = true;
    }
    else{
      first_contact = true;
      in_contact = false;
    }
  }

  void moveToPosition(float position){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = position;
    open.command.max_effort = 50.0;

    gripper_client_->sendGoal(open);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "l_gripper_force_mode");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  Gripper gripper;

  ros::waitForShutdown();

  return 0;
}
