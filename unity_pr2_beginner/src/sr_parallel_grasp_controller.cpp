#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
std::vector<std::string> joint_names{"ffj0", "ffj3", "ffj4", "mfj0", "mfj3",
                                     "mfj4", "rfj0", "rfj3", "rfj4", "lfj0",
                                     "lfj3", "lfj4", "lfj5", "thj1", "thj2",
                                     "thj3", "thj4", "thj5", "wrj1", "wrj2"};

class ParallelGraspController {
public:
  std::vector<ros::Publisher> publisher;
  ros::Publisher ffj3_pub;
  ros::Publisher mfj3_pub;
  ros::Publisher thj4_pub;
  ros::Publisher thj5_pub;
  ros::Subscriber sub;

  ParallelGraspController() {
    ros::NodeHandle nh;

    for (auto &joint : joint_names) {
      publisher.push_back(nh.advertise<std_msgs::Float64>(
          "sh_rh_" + joint + "_position_controller/command", 1));
    }

    sub = nh.subscribe("/rh_parallel_grasp_controller/command", 1,
                       &ParallelGraspController::callback, this);
  }

  void callback(const std_msgs::Float32::ConstPtr &msg) {
    // value should be opening state in cm, this will probably not be the case
    // value will be between 0.0 and 0.15
    std_msgs::Float64 f;

    for (int i = 0; i < joint_names.size(); i++) {
      if (joint_names[i] == "ffj3")
        f.data = 1.47 - msg->data * 10;
      else if (joint_names[i] == "mfj3")
        f.data = 1.57 - msg->data * 10;
      else if (joint_names[i] == "thj4")
        f.data = 1.20;
      else if (joint_names[i] == "thj5")
        f.data = 0.30 - msg->data * 7;
      else
        f.data = 0.0;

      publisher[i].publish(f);
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
