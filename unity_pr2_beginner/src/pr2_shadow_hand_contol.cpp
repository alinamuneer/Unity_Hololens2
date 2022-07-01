#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int8.h>

#include <bio_ik/bio_ik.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
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

// Converts an Eigen vector to a TF2 vector
tf2::Vector3 toTF(const Eigen::Vector3d &v) {
    return tf2::Vector3(v.x(), v.y(), v.z());
}

// Converts geometry_msgs/Point to Eigen vector
Eigen::Vector3d toVector3d(const geometry_msgs::Point &v) {
    return Eigen::Vector3d(-v.x, v.y, v.z);
}

void printVector( std::string prefix, Eigen::Vector3d vector ) {
  ROS_INFO( "%s: (%6.3f %6.3f %6.3f)",
            prefix.c_str(),
            vector.x(),
            vector.y(),
            vector.z() );
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pr2_shadow_hand_contol");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Maximum cartesian end-effector velocity
    double max_cartersion_velocity = 0;
    V(pnh.getParam("max_cartersion_velocity", max_cartersion_velocity));
    V(max_cartersion_velocity > 0.0);

    bool publish_pos;
    V(pnh.param<bool>("publish_pos", publish_pos, "false"));

    // publish hand markers
    bool show_hand_inRviz;
    V(pnh.param<bool>("show_hand_inRviz", show_hand_inRviz, "false"));
    ros::Publisher marker_pub = pnh.advertise<visualization_msgs::MarkerArray>("robot_keypoints", 1);
    ros::Publisher marker_pub2 = pnh.advertise<visualization_msgs::MarkerArray>("robot_keypoints2", 1);

    // hand keypoints from hololens
    std::mutex hololens_right_hand_keypoints_mutex;
    geometry_msgs::PoseArray hololens_right_hand_keypoints;
    // without this initialization, when there is no data coming at begining, the code will die directly
    hololens_right_hand_keypoints.poses.emplace_back();

    ros::Subscriber
    sub_hand_keypoints = nh.subscribe<geometry_msgs::PoseArray>(
            "/right_hand", 1,
            boost::function < void(
    const geometry_msgs::PoseArray &)>(
             [&](const geometry_msgs::PoseArray &data) {
                std::lock_guard <std::mutex> lock(hololens_right_hand_keypoints_mutex);
                    if (data.poses[1].position.x == 0)
                    {
                      return -1;
                    }
                    else
                      hololens_right_hand_keypoints = data;

                  // ROS_WARN_STREAM("hololens_right_hand_keypoints.poses[6].position: x:  " <<  hololens_right_hand_keypoints.poses[6].position.x
                  //     << ", y: " << hololens_right_hand_keypoints.poses[6].position.y << ", z:" << hololens_right_hand_keypoints.poses[6].position.z);


                    //   if (show_hand_inRviz)
                    //   {
                    //       int j =0;
                    //       visualization_msgs::MarkerArray hand_points;
                    //
                    //       for (auto keypoint: hololens_right_hand_keypoints.poses){
                    //         visualization_msgs::Marker point;
                    //         point.ns = std::to_string(j);
                    //         point.header.frame_id = "base_footprint";
                    //         point.type = visualization_msgs::Marker::SPHERE;
                    //         point.action = visualization_msgs::Marker::ADD;
                    //         point.color.g = 1.0f;
                    //         point.color.a = 1.0;
                    //         point.pose.orientation.w = 1.0;
                    //         point.scale.x = 0.01;
                    //         point.scale.y = 0.01;
                    //         point.scale.z = 0.01;
                    //         point.pose.position.x = -keypoint.position.x;
                    //         point.pose.position.y = keypoint.position.y;
                    //         point.pose.position.z = keypoint.position.z;
                    //         hand_points.markers.push_back(point);
                    //         j++;
                    //      }
                    //     marker_pub.publish(hand_points);
                    // }
            }));


    // publish hand joint angles by rh_trajectory_controller
    auto hand_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/hand/rh_trajectory_controller/command", 1, true);

    auto displayPublisher =
            pnh.advertise<moveit_msgs::DisplayTrajectory>(
                    "/display_hand_path", 1, true);

    // initial value setting
    std::vector<double> start_joints;
    V(pnh.getParam("start_joints", start_joints));

    std::vector<double> goal_joint_values(22, 0.0);
    std::vector<double> previous_goal_joint_values = start_joints;

    // moveit stuff
    std::string group_name = "right_fingers";
    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setMaxVelocityScalingFactor(0.3);
    auto robot_model = move_group.getRobotModel();
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    auto joint_model_group = robot_model->getJointModelGroup(group_name);

    std::string base_frame = robot_model->getModelFrame();  // base_footprint
    ROS_INFO("Reference frame: %s", base_frame.c_str());

    moveit::core::RobotState goal_state(robot_model);
    goal_state.setToDefaultValues();
    goal_state = *move_group.getCurrentState();

    if (scene->isStateColliding(goal_state, group_name, true)) {
        ROS_ERROR_STREAM("start state colliding");
        return -1;
    }

    // Keep trying to move to the start pose until it worked
    while (true) {
       ROS_INFO_STREAM("moving to start pose");
       move_group.setStartStateToCurrentState();
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

    // definition of the timeout of the IK solver
    double timeout = 0.02;
    // goal links
    std::vector <std::string> MapPositionlinks {
        "rh_thtip",
        "rh_fftip",
        "rh_mftip",
        "rh_rftip",
        "rh_thmiddle",
        "rh_ffmiddle",
        "rh_mfmiddle",
        "rh_rfmiddle"};
    std::vector <std::string> MapDirectionlinks{
        "rh_thproximal",
        "rh_ffproximal",
        "rh_mfproximal",
        "rh_rfproximal",
        "rh_thmiddle"};
    std::vector<float> MapPositionweights{1, 1, 1, 1, 0.2, 0.2, 0.2, 0.2};
    std::vector<float> MapDirectionweights{0.1, 0.1, 0.1, 0.1, 0.1};

    std::vector<Eigen::Vector3d> start_real_keypoints(8, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> previous_human_target_keypoints(8,Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> start_human_target_keypoints(8, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> goal_position(8,Eigen::Vector3d::Zero());

    // get the transform between rh_wrist (csv data are based in rh_wrist) and world
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    for (int j=0; j< start_real_keypoints.size(); j++){
        tf2::Stamped <tf2::Transform> wrist_link_tfstamped;
        geometry_msgs::TransformStamped tfGeom;
        try {
            tfGeom = tfBuffer.lookupTransform(base_frame, MapPositionlinks[j],
                                              ros::Time(0), ros::Duration(5.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        start_real_keypoints[j] = tf2::transformToEigen(tfGeom).translation();
    }
    goal_position = start_real_keypoints;

    //get the transfromation from wrist to base, because the base frame of the joint_model_group is base_footprint
    geometry_msgs::TransformStamped base_wrist_msg;
    try {
      base_wrist_msg = tfBuffer.lookupTransform(base_frame, "rh_wrist",
                                        ros::Time(0), ros::Duration(5.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return -1;
    }
    Eigen::Affine3d base_wrist_tf = tf2::transformToEigen(base_wrist_msg);

    // Run at a fixed control rate
    // Control frequency
    int frequency = 0;
    V(pnh.getParam("hand_frequency", frequency));
    V(frequency > 0);
    ros::Rate rate(frequency);

    // callback: get the arm teleoperation signal
    int teleop_start;
    int teleop_stop;
    V(pnh.getParam("teleop_signal/start", teleop_start));
    V(pnh.getParam("teleop_signal/stop", teleop_stop));

    int previous_enable = 0;

    std::mutex enable_mutex;
    int PR2_enable_msg = 0;
    ros::Subscriber enableTeleop_sub = pnh.subscribe<std_msgs::Int8>(
              "/enable_teleop", 1,
    boost::function < void(const std_msgs::Int8 &)>(
              [&](const std_msgs::Int8 &enableTeleopMsg) {
                  std::lock_guard <std::mutex> lock(enable_mutex);
                  PR2_enable_msg = enableTeleopMsg.data;
              }));

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
          bio_ik::BioIKKinematicsQueryOptions ik_options;
          ik_options.replace = true;
          ik_options.fixed_joints = {"rh_WRJ1", "rh_WRJ2"};
          ik_options.return_approximate_solution = true;

          for (int j = 0; j < MapPositionlinks.size(); j++) {
              ik_options.goals.emplace_back(
                      new bio_ik::PositionGoal(MapPositionlinks[j], toTF(goal_position[j]), MapPositionweights[j]));

              visualization_msgs::MarkerArray hand_points;
              if (show_hand_inRviz)
              {
                  visualization_msgs::Marker point;
                  point.ns = std::to_string(j);
                  point.header.frame_id = "base_footprint";
                  point.type = visualization_msgs::Marker::SPHERE;
                  point.action = visualization_msgs::Marker::ADD;
                  point.color.g = 1.0f;
                  point.color.a = 1.0;
                  point.pose.orientation.w = 1.0;
                  point.scale.x = 0.01;
                  point.scale.y = 0.01;
                  point.scale.z = 0.01;
                  point.pose.position.x = goal_position[j].x();
                  point.pose.position.y = goal_position[j].y();
                  point.pose.position.z = goal_position[j].z();
                  hand_points.markers.push_back(point);
              }
              marker_pub.publish(hand_points);
          }

          // set ik solver
          bool found_ik = goal_state.setFromIK(
                  joint_model_group,           // active Shadow joints
                  EigenSTL::vector_Isometry3d(), // no explicit poses here
                  std::vector<std::string>(),
                  timeout,
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
            if (!found_ik) {
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

                    // for (int j = 0; j <goal_joint_values.size(); j++) {
                    //    ROS_WARN_STREAM("goal_joint_values IS " << goal_joint_values[j]);}

                    trajectory_msgs::JointTrajectory hand_joint_trajectory;
                    hand_joint_trajectory.points.emplace_back();
                    hand_joint_trajectory.joint_names = joint_model_group->getVariableNames();
                    for (int j=0; j <goal_joint_values.size(); j++)
                    {
                        double vel =
                                std::abs(goal_joint_values[j] - previous_goal_joint_values[j]) * frequency;
                        //ROS_WARN_STREAM("goal_joint_values[j]: "<< goal_joint_values[j] << ", previous_goal_joint_values[j]: "<<
                        // previous_goal_joint_values[j] << ", vel: " << vel);

                        double factor = std::min(1.0, 0.5 / vel);
                        double fine_goal_joint_value =
                                goal_joint_values[j] * factor + (1 - factor) * previous_goal_joint_values[j];

                        //ROS_WARN_STREAM("factor: "<< factor << ", j: "<< j << ", fine_goal_joint_value: " << fine_goal_joint_value);
                        hand_joint_trajectory.points.back().positions.emplace_back(fine_goal_joint_value);
                        hand_joint_trajectory.points.back().time_from_start = ros::Duration(1.0 / frequency);
                    }

                    // for (auto &n : arm_joint_model_group->getVariableNames()) {
                    //     ROS_INFO_STREAM("joint " << n );}
                    // for (auto &n : joint_model_group->getVariableNames()) {
                    //     ROS_INFO_STREAM("joint " << n );}

                    if (publish_pos)
                        hand_pub.publish(hand_joint_trajectory);

                    goal_state.setJointGroupPositions(joint_model_group, goal_joint_values);

                    // visualization
                    {
                        moveit_msgs::DisplayTrajectory dtm;
                        moveit::core::robotStateToRobotStateMsg(goal_state, dtm.trajectory_start);

                        trajectory_msgs::JointTrajectory arm_joint_trajectory;
                        arm_joint_trajectory.joint_names = joint_model_group->getVariableNames();
                        arm_joint_trajectory.points.emplace_back();
                        // for (auto &name : arm_joint_trajectory.joint_names)
                        //     ROS_WARN_STREAM("JOINT name is " << name);
                        for (auto goal_joint_value:goal_joint_values)
                                arm_joint_trajectory.points.back().positions.emplace_back(
                                        goal_joint_value);

                        dtm.trajectory.emplace_back();
                        dtm.trajectory.back().joint_trajectory = arm_joint_trajectory;
                        dtm.trajectory.back().joint_trajectory.points.back().time_from_start =
                                ros::Duration(2.0);
                        ros::Duration(0.01).sleep();
                        displayPublisher.publish(dtm);
                    }
                    previous_goal_joint_values = goal_joint_values;
                }
      }

      // Wait for next control cycle
       bool timing_ok = rate.sleep();
       if (!timing_ok) {
           ROS_ERROR_STREAM("timing error, control may be unstable");
       }

        // Apply cartesian workspace and velocity, acceleration constraints
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
        std::vector<Eigen::Vector3d> human_target_keypoints(8,Eigen::Vector3d::Zero());

        {
            std::lock_guard <std::mutex> lock_tag(hololens_right_hand_keypoints_mutex);
            // wrist
            //human_target_keypoints[0] = toVector3d(hololens_right_hand_keypoints.poses[1].position);

            // thumb tip, proxiaml, metarcapal
            human_target_keypoints[0] = toVector3d(hololens_right_hand_keypoints.poses[6].position);
            human_target_keypoints[1] = toVector3d(hololens_right_hand_keypoints.poses[11].position);
            human_target_keypoints[2] = toVector3d(hololens_right_hand_keypoints.poses[16].position);
            human_target_keypoints[3] = toVector3d(hololens_right_hand_keypoints.poses[21].position);

            // middle link
            human_target_keypoints[4] = toVector3d(hololens_right_hand_keypoints.poses[4].position);
            human_target_keypoints[5] = toVector3d(hololens_right_hand_keypoints.poses[9].position);
            human_target_keypoints[6] = toVector3d(hololens_right_hand_keypoints.poses[14].position);
            human_target_keypoints[7] = toVector3d(hololens_right_hand_keypoints.poses[19].position);

            // metarcapal
            //human_target_keypoints[9] = toVector3d(hololens_right_hand_keypoints.poses[3].position);
            //human_target_keypoints[10] = toVector3d(hololens_right_hand_keypoints.poses[7].position);
            //human_target_keypoints[11] = toVector3d(hololens_right_hand_keypoints.poses[12].position);
            //human_target_keypoints[12] = toVector3d(hololens_right_hand_keypoints.poses[17].position);

              if (show_hand_inRviz)
              {
                  int j =0;
                  visualization_msgs::MarkerArray hand_points;

                  for (auto keypoint: human_target_keypoints){
                    visualization_msgs::Marker point;
                    point.ns = std::to_string(j);
                    point.header.frame_id = "base_footprint";
                    point.type = visualization_msgs::Marker::SPHERE;
                    point.action = visualization_msgs::Marker::ADD;
                    point.color.r = 1.0f;
                    point.color.a = 1.0;
                    point.pose.orientation.w = 1.0;
                    point.scale.x = 0.01;
                    point.scale.y = 0.01;
                    point.scale.z = 0.01;
                    point.pose.position.x = keypoint.x();
                    point.pose.position.y = keypoint.y();
                    point.pose.position.z = keypoint.z();
                    hand_points.markers.push_back(point);
                    j++;
                 }
                marker_pub2.publish(hand_points);
            }

        }

        if ((enable_PR2==teleop_start && (previous_enable==teleop_stop)) || enable_PR2==0 || iteration==0)
        {
             geometry_msgs::TransformStamped base_palm_tf;
             for (int j=0; j< start_real_keypoints.size(); j++){
                 tf2::Stamped <tf2::Transform> wrist_link_tfstamped;
                 geometry_msgs::TransformStamped tfGeom;
                 try {
                     tfGeom = tfBuffer.lookupTransform(base_frame, MapPositionlinks[j],
                                                       ros::Time(0), ros::Duration(5.0));
                 }
                 catch (tf2::TransformException &ex) {
                     ROS_WARN("%s", ex.what());
                     ros::Duration(1.0).sleep();
                 }
                 start_real_keypoints[j] = tf2::transformToEigen(tfGeom).translation();
             }

             previous_human_target_keypoints = human_target_keypoints;
             start_human_target_keypoints = human_target_keypoints;
        }
        // velocity constraints
        if (enable_PR2!=teleop_stop)
        {
            for (int j=0; j < human_target_keypoints.size(); j++) {
                Eigen::Vector3d cartersion_velocity = (human_target_keypoints[j] - previous_human_target_keypoints[j]) *  frequency;
                if (cartersion_velocity.norm() > max_cartersion_velocity) {
                    ROS_WARN_STREAM("Joint vel is " <<  cartersion_velocity.norm() << ". TOO BIG");
                    cartersion_velocity = cartersion_velocity.normalized() * max_cartersion_velocity;
                }

                human_target_keypoints[j] = previous_human_target_keypoints[j] + cartersion_velocity * (1.0 / frequency);
                previous_human_target_keypoints[j] = human_target_keypoints[j];
                goal_position[j] = start_real_keypoints[j] + (human_target_keypoints[j] - start_human_target_keypoints[j]);

                // if (j==3)
                // {
                //   printVector("human_target_keypoints", human_target_keypoints[j]);
                //   printVector("previous_human_target_keypoints", previous_human_target_keypoints[j]);
                //   printVector("new human_target_keypoints", human_target_keypoints[j]);
                //   printVector("start_real_keypoints", start_real_keypoints[j]);
                //   printVector("start_human_target_keypoints", start_human_target_keypoints[j]);
                //   printVector("goal_position", goal_position[j]);
                // }
            }
        }

       previous_enable = enable_PR2;
    }
    return 0;
}
