#include <ros/console.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <signal.h>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>
#include <numeric>

#include <bio_ik/bio_ik.h>
#include <geometry_msgs/Vector3.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>

// Helper macro, throws an exception if a statement fails
#define VXSTR(s) VSTR(s)
#define VSTR(s) #s
#define V(x)                                                                   \
  if (!(x)) {                                                                  \
    ros::Duration(1.0).sleep();                                                \
    throw std::runtime_error(VXSTR(x));                                        \
  }

// Converts and Eigen vector to a TF2 vector
tf2::Vector3 toTF(const Eigen::Vector3d &v) {
    return tf2::Vector3(v.x(), v.y(), v.z());
}

tf2::Quaternion QuaterniontoTF(const Eigen::Quaterniond &v) {
    return tf2::Quaternion(v.x(), v.y(), v.z(), v.w());
}

void printVector( std::string prefix, Eigen::Vector3d vector ) {
  ROS_INFO( "%s: (%6.3f %6.3f %6.3f)",
            prefix.c_str(),
            vector.x(),
            vector.y(),
            vector.z() );
}

void printTransform( std::string prefix, tf2::Transform trafo ) {
  ROS_INFO( "%s: origin: (%6.3f %6.3f %6.3f) quat: (%6.4f %6.4f %6.4f %6.4f)",
            prefix.c_str(),
            trafo.getOrigin().x(),
            trafo.getOrigin().y(),
            trafo.getOrigin().z(),
            trafo.getRotation().x(),
            trafo.getRotation().y(),
            trafo.getRotation().z(),
            trafo.getRotation().w() );
}

double clip(double x, double maxv = 0, double minv = 0) {
    if (x > maxv)
        x = maxv;
    if (x < minv)
        x = minv;
    return x;
}

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "pr2_arms_control"); // 1=no NoSigintHandler
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(5);
    spinner.start();

    // Maximum joint-space end-effector acceleration
    double max_joint_acceleration = 0.0;
    V(pnh.getParam("max_joint_acceleration", max_joint_acceleration));
    V(max_joint_acceleration > 0.0);

    // std::vector<double> max_joint_velocity = {2.1, 2.1, 3.0, 3.3, 3.6};

    // Maximum cartesian end-effector velocity
    double max_position_velocity = 0;
    V(pnh.getParam("max_position_velocity", max_position_velocity));
    V(max_position_velocity > 0.0);

    double max_rotation_velocity = 0;
    V(pnh.getParam("max_rotation_velocity", max_rotation_velocity));
    V(max_rotation_velocity > 0.0);

    double max_position_acceleration = 0;
    V(pnh.getParam("max_position_acceleration", max_position_acceleration));
    V(max_position_acceleration > 0.0);

    // Control frequency
    int frequency = 0;
    V(pnh.getParam("arm_frequency", frequency));
    V(frequency > 0);
    double max_rotation_angle = max_rotation_velocity / frequency;

    double velocity_fd_factor;
    double velocity_ff_factor;
    // Maximum joint-space velocity, just for safety. You should mainly rely on
    V(pnh.getParam("feedforward_velocity_factor", velocity_ff_factor));
    V(pnh.getParam("feedback_velocity_factor", velocity_fd_factor));

    bool publish_pos;
    V(pnh.param<bool>("publish_pos", publish_pos, "false"));

    bool publish_wrist;
    V(pnh.param<bool>("publish_wrist", publish_wrist, "false"));

    std::string group_name;
    V(pnh.getParam("group_name", group_name));

    std::string eef_link;
    V(pnh.getParam("eef_link", eef_link));

    // Init MoveIt stuff
    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setMaxVelocityScalingFactor(0.3);
    auto robot_model = move_group.getRobotModel();
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    auto joint_model_group = robot_model->getJointModelGroup(group_name);

    const std::vector< std::string > move_group_joint_names = joint_model_group->getVariableNames();
    std::string base_frame = robot_model->getModelFrame();  // base_footprint
    //ROS_INFO("Reference frame: %s", base_frame.c_str());

    moveit::core::RobotState goal_state(robot_model);
    goal_state.setToDefaultValues();
    goal_state = *move_group.getCurrentState();
    ros::Duration(1.0).sleep();

    if (scene->isStateColliding(goal_state, group_name, true)) {
        ROS_ERROR_STREAM("start state colliding");
        return -1;
    }

    //callback: goal position
    std::string goal_topic;
    V(pnh.getParam("goal_topic", goal_topic));

    std::mutex hand_transform_mutex;
    Eigen::Vector3d pAvg = Eigen::Vector3d::Zero();
    Eigen::Vector3d hololens_wrist_unity_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_hololens_wrist_unity_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond hololens_wrist_unity_rot = Eigen::Quaterniond(1, 0, 0, 0);
    ros::Subscriber
    sub_azure = nh.subscribe<geometry_msgs::TransformStamped>(
            goal_topic, 1,
            boost::function < void(
    const geometry_msgs::TransformStamped &)>(
             [&](const geometry_msgs::TransformStamped &data) {
                std::lock_guard <std::mutex> lock(hand_transform_mutex);
                    hololens_wrist_unity_pos.x() = data.transform.translation.x;
                    hololens_wrist_unity_pos.y() = data.transform.translation.y;
                    hololens_wrist_unity_pos.z() = data.transform.translation.z;
                    hololens_wrist_unity_rot.x() = data.transform.rotation.x;
                    hololens_wrist_unity_rot.y() = data.transform.rotation.y;
                    hololens_wrist_unity_rot.z() = data.transform.rotation.z;
                    hololens_wrist_unity_rot.w() = data.transform.rotation.w;
            }));

      // callback: get the arm teleoperation signal
      int teleop_start;
      int teleop_stop;
      V(pnh.getParam("teleop_signal/start", teleop_start));
      V(pnh.getParam("teleop_signal/stop", teleop_stop));

      std::mutex enable_mutex;
      int PR2_enable_msg = 0;
      ros::Subscriber enableTeleop_sub = pnh.subscribe<std_msgs::Int8>(
                "/enable_teleop", 1,
      boost::function < void(const std_msgs::Int8 &)>(
                [&](const std_msgs::Int8 &enableTeleopMsg) {
                    std::lock_guard <std::mutex> lock(enable_mutex);
                    PR2_enable_msg = enableTeleopMsg.data;
                }));

    // callback: get realtime joint states
    std::mutex joint_mutex;
    std::vector<double> r_arm_joints(7, 0.0);
    ros::Subscriber sub_joint_states = pnh.subscribe<sensor_msgs::JointState>(
              "/joint_states", 1,
    boost::function < void(const sensor_msgs::JointState &)>(
              [&](const sensor_msgs::JointState &jointMsg) {
                  std::lock_guard <std::mutex> lock(joint_mutex);
               if (std::find(jointMsg.name.begin(), jointMsg.name.end(), "fl_caster_rotation_joint")!=jointMsg.name.end()){
                  r_arm_joints[0] = jointMsg.position[18];
                  r_arm_joints[1] = jointMsg.position[19];
                  r_arm_joints[2] = jointMsg.position[17];
                  r_arm_joints[3] = jointMsg.position[21];
                  r_arm_joints[4] = jointMsg.position[20];
                  }
                else{
                  r_arm_joints[5] = jointMsg.position[22];
                  r_arm_joints[6] = jointMsg.position[23];}
              }));

    std::string arm_controller_name;
    V(pnh.getParam("arm_controller_name", arm_controller_name));
    auto arm_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/" +
              arm_controller_name + "/command", 1);
    auto r_wrist_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/hand/rh_trajectory_controller/command", 1, true);

    std::string display_trajectory_name;
    V(pnh.getParam("display_trajectory_name", display_trajectory_name));
    auto displayPublisher =
            pnh.advertise<moveit_msgs::DisplayTrajectory>("/" +
                    display_trajectory_name, 1, true);

    ros::Publisher pose_publisher =
            pnh.advertise<geometry_msgs::PoseArray>(
                    "/tag_goals", 1, true);

    //  set intial position and velocity, acceleration
    Eigen::Vector3d start_position = Eigen::Vector3d::Zero();
    V(pnh.getParam("start_position/x", (double &) start_position.x()));
    V(pnh.getParam("start_position/y", (double &) start_position.y()));
    V(pnh.getParam("start_position/z", (double &) start_position.z()));

    Eigen::Quaterniond start_rotation_q(1, 0, 0, 0);
    V(pnh.getParam("start_rotation_q/x", (double &) start_rotation_q.x()));
    V(pnh.getParam("start_rotation_q/y", (double &) start_rotation_q.y()));
    V(pnh.getParam("start_rotation_q/z", (double &) start_rotation_q.z()));
    V(pnh.getParam("start_rotation_q/w", (double &) start_rotation_q.w()));

    std::vector<double> start_joints;
    V(pnh.getParam("start_joints", start_joints));

    Eigen::Vector3d goal_position = start_position;
    Eigen::Vector3d position_velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation_matrix_velocity = Eigen::Matrix3d::Zero();
    Eigen::Quaterniond goal_rotation_q = start_rotation_q;
    std::vector<double> goal_joint_values(7, 0.0);

    std::vector<double> previous_goal_joint_values = start_joints;
    Eigen::Vector3d previous_goal_position = goal_position;
    Eigen::Vector3d previous_position_velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d previous_goal_rotation_matrix = goal_rotation_q.normalized().toRotationMatrix();
    std::vector<double> previous_joint_velocity = {0,0,0,0,0};

    std::vector<double> wrist_goal_array;

    // todo: static tf from hololens camera to pr2 base_footprint

    Eigen::Affine3d pr2base_hololenscamera;
    if (group_name == "right_arm")
      pr2base_hololenscamera  = Eigen::Affine3d(Eigen::Quaterniond(1.0, 0, 0, 0.0));
    else
      pr2base_hololenscamera  = Eigen::Affine3d(Eigen::Quaterniond(0.7071068, 0, 0.7071068, 0.0)) * Eigen::Affine3d(Eigen::Quaterniond(-0.7071068, 0.7071068, 0.0, 0.0));

    // static tf from torso_lift)link to pr2 base_footprint
    tf2_ros::TransformBroadcaster tf_broadcaster;
    // listen to the tf depth_camera_link with respect to camera_base/world
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped base_torso_msg;
    try {
      base_torso_msg = tfBuffer.lookupTransform(base_frame, "torso_lift_link",
                                        ros::Time(0), ros::Duration(5.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return -1;
    }
    Eigen::Affine3d base_torso_tf = tf2::transformToEigen(base_torso_msg);

    // Run at a fixed control rate
    ros::Rate rate(frequency);

    int previous_enable = 0;
    Eigen::Vector3d start_p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d start_m = Eigen::Matrix3d::Zero();

    Eigen::Affine3d start_pose_affine = Eigen::Affine3d::Identity();
    // Keep trying to move to the start pose until it worked
    while (true) {
       ROS_INFO_STREAM("moving to start pose");
       move_group.setStartStateToCurrentState();
       // V(move_group.setJointValueTarget(goal_state));
       V(move_group.setJointValueTarget(start_joints));
       auto success = move_group.move();

       if (success) {
           ROS_INFO_STREAM("success");
           goal_state.setJointGroupPositions(joint_model_group, start_joints);
           break;
       } else {
           ROS_INFO_STREAM("failed to move to start pose");
           ros::Duration(1.0).sleep();
       }
    }

    // Control loop
    for (size_t iteration = 0;; iteration++) {
        ros::Time begin = ros::Time::now();

        int enable_PR2=0;
        {
          std::lock_guard <std::mutex> lock_pedal(enable_mutex);
          enable_PR2 = PR2_enable_msg;
        }

        if (enable_PR2!=teleop_stop)
        // ik calculation
        {
            // Setup BioIK goals
            bio_ik::BioIKKinematicsQueryOptions ik_options;
            ik_options.replace = true;
            ik_options.return_approximate_solution = true;

            // printVector("goal_position ik", goal_position);
            ik_options.goals.emplace_back(
                    new bio_ik::PositionGoal(eef_link, toTF(goal_position)));
            ik_options.goals.emplace_back(
                   new bio_ik::OrientationGoal(eef_link, QuaterniontoTF(goal_rotation_q)));

            // printVector("goal position", goal_position);

            // ik_options.goals.emplace_back(
            //             new bio_ik::MinimalDisplacementGoal(1.0, false));
            ik_options.goals.emplace_back(new bio_ik::RegularizationGoal(0.5));

            //for (int j = 0; j <goal_joint_values.size(); j++) {
            //    ROS_WARN_STREAM("before ik JOINT IS " << goal_joint_values[j]);}
            // Call BioIK
            bool ik_ok = goal_state.setFromIK(
                    joint_model_group, EigenSTL::vector_Isometry3d(),
                    std::vector<std::string>(),
                    (0.7 / frequency),
                    moveit::core::GroupStateValidityCallbackFn(
                            [&](robot_state::RobotState *state,
                                const robot_state::JointModelGroup *group,
                                const double *ik_solution) {
                                state->setJointGroupPositions(group, ik_solution);
                                state->update();
                                bool collision = (bool) (scene->isStateColliding(
                                        *state, group->getName(), true));
                                if (collision) {
                                    ROS_ERROR_STREAM("collision, solution rejected");
                                }
                                return !collision;
                            }),
                    ik_options);

            // Exit if IK failed
            if (!ik_ok) {
                ROS_ERROR_STREAM("ik failed");
            }
            else{
                    goal_state.copyJointGroupPositions(joint_model_group, goal_joint_values);
                    for (int j = 0; j <goal_joint_values.size(); j++) {
                        if (M_PI < goal_joint_values[j]){
                            goal_joint_values[j] = goal_joint_values[j] - 2 * M_PI;
                        }
                        if (-M_PI >= goal_joint_values[j]){
                            goal_joint_values[j] = goal_joint_values[j] + 2 * M_PI;
                        }
                    }

                    std::vector<double> current_joint_values(7, 0.0);
                    {
                        std::lock_guard <std::mutex> lock_joint(joint_mutex);
                        current_joint_values = r_arm_joints;
                    }

                    std::vector<double> joint_velocity(5, 0.0);

                    trajectory_msgs::JointTrajectory wrist_goal;
                    wrist_goal.points.emplace_back();
                    wrist_goal.joint_names.emplace_back("rh_WRJ2");
                    wrist_goal.joint_names.emplace_back("rh_WRJ1");

                    // for (int j = 0; j <goal_joint_values.size(); j++) {
                    //    ROS_WARN_STREAM("goal_joint_values IS " << goal_joint_values[j]);}

                    // Apply joint-space velocity and acceleration limit for arm
//                    {
//                        for (int j = 0; j < goal_joint_values.size(); j++) {
//                            if (j < 5) {
//                                //ROS_INFO_STREAM("goal_joint_values " <<goal_joint_values[j] << " previous_goal_joint_values " << previous_goal_joint_values[j] );
//                                double joint_feedforward_diff = goal_joint_values[j] - previous_goal_joint_values[j];
//                                double joint_feedback_diff = goal_joint_values[j] - current_joint_values[j];
//                                double vel = joint_feedforward_diff * frequency * velocity_ff_factor
//                                             + joint_feedback_diff * velocity_fd_factor;
//
//                                //ROS_INFO_STREAM("vel is "<< vel);
//                                if (M_PI < vel)
//                                    vel = vel - 2 * M_PI;
//                                if (-M_PI > vel)
//                                    vel = vel + 2 * M_PI;
//                                vel = clip(vel, max_joint_velocity[j], -max_joint_velocity[j]);
//                                //ROS_INFO_STREAM("vel clip is "<< vel);
//
//                                // add acceleration constraints
//                                double previous_vel = previous_joint_velocity[j];
//                                //ROS_INFO_STREAM("prev vel is "<< previous_vel);
//                                double acceleration = (vel - previous_vel) * frequency;
//                                //ROS_INFO_STREAM("acc is "<< acceleration);
//                                if (std::abs(acceleration) > max_joint_acceleration) {
//                                    double acceleration_factor = max_joint_acceleration / std::abs(acceleration);
//                                    vel = previous_vel + acceleration * acceleration_factor * (1.0 / frequency);
//                                    // ROS_INFO_STREAM("joint acceleration is " << acceleration << ", too big!");
//                                }
//                                //ROS_INFO_STREAM("vel inter "<< vel);
//                                goal_joint_values[j] = previous_goal_joint_values[j] + vel * (1.0 / frequency);
//                            r_arm_vel_pub    //ROS_INFO_STREAM("goal_joint_values " <<goal_joint_values[j] << " previous_goal_joint_values " << previous_goal_joint_values[j] );
//                                joint_velocity[j] = vel;
//                            } else {
//                                double wrist_vel =
//                                        std::abs(goal_joint_values[j] - previous_goal_joint_values[j]) * frequency;
//                                double factor = std::min(1.0, 0.5 / wrist_vel);
//                                double wrist_goal_joint_value =
//                                        goal_joint_values[j] * factor + (1 - factor) * previous_goal_joint_values[j];
//                                wrist_goal.points.back().positions.emplace_back(wrist_goal_joint_value);
//                            }
//                        }
//                    }
//                    // for (auto &n : arm_joint_model_group->getVariableNames()) {
                    //     ROS_INFO_STREAM("joint " << n );}
                    //for (auto &n : joint_model_group->getVariableNames()) {
                    //     ROS_INFO_STREAM("joint " << n );}

                    if (group_name == "right_arm")
                    {
                      if (publish_pos)
                      {
                          trajectory_msgs::JointTrajectory arm_joint_trajectory;
                          arm_joint_trajectory.points.emplace_back();
                          for (int j=0; j <5; j++) {
                              arm_joint_trajectory.joint_names.emplace_back(move_group_joint_names[j]);
                              arm_joint_trajectory.points.back().positions.emplace_back(
                                  goal_joint_values[j]);
                          }
                          arm_joint_trajectory.points.back().time_from_start =
                                  ros::Duration(1.0 / frequency);
                          arm_trajectory_pub.publish(arm_joint_trajectory);
                      }
                      if (publish_wrist){
                        for (int j=5; j <7; j++) {
                            wrist_goal.points.back().positions.emplace_back(
                                goal_joint_values[j]);
                        }
                          wrist_goal.points.back().time_from_start = ros::Duration(1.0 / frequency);
                          r_wrist_pub.publish(wrist_goal);
                      }
                    }
                    else
                    {
                      if (publish_pos)
                      {
                          trajectory_msgs::JointTrajectory arm_joint_trajectory;
                          arm_joint_trajectory.joint_names = joint_model_group->getVariableNames();
                          arm_joint_trajectory.points.emplace_back();
                          for (int j=0; j <7; j++) {
                              //arm_joint_trajectory.joint_names.emplace_back(move_group_joint_names[j]);
                              arm_joint_trajectory.points.back().positions.emplace_back(
                                  goal_joint_values[j]);
                          }
                          arm_joint_trajectory.points.back().time_from_start =
                                  ros::Duration(1.0 / frequency);
                          arm_trajectory_pub.publish(arm_joint_trajectory);
                      }
                    }

                    goal_state.setJointGroupPositions(joint_model_group, goal_joint_values);


                    //goal_state.copyJointGroupPositions(joint_model_group, goal_joint_values);

                    // visualization
                    {
                        moveit_msgs::DisplayTrajectory dtm;
                        moveit::core::robotStateToRobotStateMsg(goal_state, dtm.trajectory_start);

                        trajectory_msgs::JointTrajectory arm_joint_trajectory;
                        arm_joint_trajectory.joint_names = joint_model_group->getVariableNames();
                        arm_joint_trajectory.points.emplace_back();
                        // for (auto &name : arm_joint_trajectory.joint_names)
                        //     ROS_WARN_STREAM("JOINT name is " << name);
                        for (int j=0; j < goal_joint_values.size(); j++) {
                                // arm_joint_trajectory.points.back().positions.emplace_back(
                                //         goal_state.getVariablePosition(name));
                                arm_joint_trajectory.points.back().positions.emplace_back(
                                        goal_joint_values[j]);
                        }

                        dtm.trajectory.emplace_back();
                        dtm.trajectory.back().joint_trajectory = arm_joint_trajectory;
                        dtm.trajectory.back().joint_trajectory.points.back().time_from_start =
                                ros::Duration(2.0);
                        ros::Duration(0.01).sleep();
                        displayPublisher.publish(dtm);
                    }

                    previous_joint_velocity = joint_velocity;
                    previous_goal_joint_values = goal_joint_values;

                }
      }
      // Wait for next control cycle
       bool timing_ok = rate.sleep();
       if (!timing_ok) {
           ROS_ERROR_STREAM("timing error, control may be unstable");
       }

        if (enable_PR2!=teleop_stop){
          geometry_msgs::PoseArray pose_goal;
          pose_goal.header.frame_id = "base_footprint";
          geometry_msgs::TransformStamped base_palm_tf;

          try {
                base_palm_tf = tfBuffer.lookupTransform("base_footprint", eef_link,
                                                            ros::Time(0), ros::Duration(2.0));
          }
          catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                return -1;
          }

        // Eigen::Affine3d current_pose_affine = tf2::transformToEigen(base_palm_tf);
        // Eigen::Quaterniond qf_current(current_pose_affine.linear());
        }

        if (enable_PR2==0 || enable_PR2==teleop_stop){
           previous_joint_velocity = {0, 0, 0, 0, 0};
           previous_position_velocity= Eigen::Vector3d::Zero();
        }

        // Apply cartesian workspace and velocity, acceleration constraints
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
        {
            std::lock_guard <std::mutex> lock_tag(hand_transform_mutex);
            Eigen::Affine3d hololens_right_wrist = Eigen::Affine3d::Identity();
            hololens_right_wrist.translation() = hololens_wrist_unity_pos;
            hololens_right_wrist.linear() = hololens_wrist_unity_rot.toRotationMatrix();

            Eigen::Affine3d pr2base_palm =  pr2base_hololenscamera * hololens_right_wrist;
            // std::cout << pr2base_palm.matrix() << std::endl;

            p = pr2base_palm.translation();
            m = pr2base_palm.linear();
        }

    // Eigen::Affine3d pr2base_urworld  = Eigen::Affine3d(Eigen::Quaterniond(0.7071068, 0.0, 0.0, -0.7071068));
        // initialize previous goal position when restart the teleoperation
        if ((enable_PR2==teleop_start && (previous_enable==teleop_stop)) || enable_PR2==0 || iteration==0){
             geometry_msgs::TransformStamped base_palm_tf;
            try {
                base_palm_tf = tfBuffer.lookupTransform("base_footprint", eef_link,
                                                            ros::Time(0), ros::Duration(1.0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                return -1;
          }

           start_pose_affine = tf2::transformToEigen(base_palm_tf);

           previous_goal_position = p;
           previous_position_velocity = Eigen::Vector3d::Zero();
           previous_goal_rotation_matrix = m;
           previous_joint_velocity = {0, 0, 0, 0, 0};

           start_p = p;
           start_m = m;
        }

        // linear constraints
        if (enable_PR2!=teleop_stop){
        {
             position_velocity = (p - previous_goal_position) *  frequency;
             if (position_velocity.norm() > max_position_velocity) {
                 ROS_WARN_STREAM("Linear vel is " <<  position_velocity.norm() << ". TOO BIG");
                 position_velocity = position_velocity.normalized() * max_position_velocity;
             }

             Eigen::Vector3d acceleration = (position_velocity - previous_position_velocity) * frequency;
             if (acceleration.norm() > max_position_acceleration) {
                 acceleration = acceleration.normalized() * max_position_acceleration;
             }

             position_velocity = previous_position_velocity + acceleration * (1.0 / frequency);
//            p.x() = previous_goal_position.x() + position_velocity.x() * (1.0 / frequency) * 0.3;
//            p.y() = previous_goal_position.y() + position_velocity.y() * (1.0 / frequency) * 0.3;
//            p.z() = previous_goal_position.z() + position_velocity.z() * (1.0 / frequency);
             p = previous_goal_position + position_velocity * (1.0 / frequency);
        }

        {
            // angular constraints
            Eigen::Matrix3d rotation_matrix_diff = previous_goal_rotation_matrix.inverse() * m;
            Eigen::Quaterniond qf(m);
            Eigen::Quaterniond previous_qf(previous_goal_rotation_matrix);

            double angle_diff = previous_qf.angularDistance(qf);
            double vel_factor = std::min(1.0, max_rotation_angle / angle_diff);
            Eigen::Quaterniond qf_inter = previous_qf.slerp(vel_factor, qf);
            m = qf_inter.toRotationMatrix();
        }

        // not enable_PR2, don't update goal and goal_state
        Eigen::Matrix3d goal_m_diff = start_m.inverse() * m;
        Eigen::Matrix3d m_goal = start_pose_affine.linear() * goal_m_diff;
        Eigen::Quaterniond qf_goal(m_goal);
        goal_rotation_q = qf_goal.normalized();
        goal_position = start_pose_affine.translation() + (p - start_p);

        geometry_msgs::PoseArray pose_goal;

        pose_goal.poses.emplace_back();
        pose_goal.poses.back().position.x = goal_position.x();
        pose_goal.poses.back().position.y = goal_position.y();
        pose_goal.poses.back().position.z = goal_position.z();
        pose_goal.poses.back().orientation.z = goal_rotation_q.z();
        pose_goal.poses.back().orientation.x = goal_rotation_q.x();
        pose_goal.poses.back().orientation.y = goal_rotation_q.y();
        pose_goal.poses.back().orientation.w = goal_rotation_q.w();
        pose_publisher.publish(pose_goal);

        previous_goal_position = p;
        previous_position_velocity = position_velocity;
        previous_goal_rotation_matrix = m;
       }
       previous_enable = enable_PR2;
       // ROS_INFO_STREAM("goal cal time: " <<  "\n" << ros::Time::now() - begin);
    }
    return 0;
}
