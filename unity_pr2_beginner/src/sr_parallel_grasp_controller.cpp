#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
std::vector<std::string> joint_names{"ffj0", "ffj3", "ffj4", "mfj0", "mfj3",
                                     "mfj4", "rfj0", "rfj3", "rfj4", "lfj0",
                                     "lfj3", "lfj4", "lfj5", "thj1", "thj2",
                                     "thj3", "thj4", "thj5", "wrj1", "wrj2"};

class ParallelGraspController {
public:
  std::vector<ros::Publisher> publisher;
  ros::Publisher pub;
  ros::Subscriber sub;

  ParallelGraspController() {
    ros::NodeHandle nh;
    pub = nh.advertise<trajectory_msgs::JointTrajectory>("rh_trajectory_controller/command",1);
    sub = nh.subscribe("/rh_parallel_grasp_controller/command", 1,
                       &ParallelGraspController::callback, this);
  }

  void callback(const std_msgs::Float32::ConstPtr &msg) {
    // value should be opening state in cm, this will probably not be the case
    // value will be between 0.0 and 0.15
    // std_msgs::Float64 f;

    trajectory_msgs::JointTrajectory arm_joint_trajectory;
    arm_joint_trajectory.points.emplace_back();

    for (int i = 0; i < joint_names.size(); i++) {
      if (joint_names[i] == "ffj3"){
        arm_joint_trajectory.joint_names.emplace_back("rh_FFJ3");
        arm_joint_trajectory.points.back().positions.emplace_back(1.47 - msg->data * 10);
}
      else if (joint_names[i] == "mfj3"){
        arm_joint_trajectory.joint_names.emplace_back("rh_MFJ3");
        arm_joint_trajectory.points.back().positions.emplace_back(1.57 - msg->data * 10);
}
      else if (joint_names[i] == "thj4"){
                arm_joint_trajectory.joint_names.emplace_back("rh_THJ4");
        arm_joint_trajectory.points.back().positions.emplace_back(1.20);
       }
      else if (joint_names[i] == "thj5"){
        arm_joint_trajectory.joint_names.emplace_back("rh_THJ5");
        arm_joint_trajectory.points.back().positions.emplace_back(0.30 - msg->data * 7);
}
      else
        continue;
      arm_joint_trajectory.points.back().time_from_start =
                                  ros::Duration(1.0 / 30);
      pub.publish(arm_joint_trajectory);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "parallel_grasp_controller");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ParallelGraspController pgc;

  ros::waitForShutdown();

  return 0;
}
