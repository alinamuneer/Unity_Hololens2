// Quickly generate large image datasets from robot models.
// (c) 2020 Shuang Li

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <bio_ik/bio_ik.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <opencv2/opencv.hpp>

#define TEXT(str...) #str


int main(int argc, char **argv) {
    ros::init(argc, argv, "opengl_shadow_bioik_dataset_generator", 0);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    OffscreenContext opengl_context;
    //glewInit();
    RobotDatasetGenerator robot_image_dataset_generator;

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

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

    bool show_hand_inRviz;

    pnh.getParam("show_hand_inRviz", show_hand_inRviz);
    visualization_msgs::MarkerArray hand_points;
    ros::Publisher marker_pub = pnh.advertise<visualization_msgs::MarkerArray>("robot_keypoints", 1);
    ros::Publisher hand_pub = pnh.advertise<trajectory_msgs::JointTrajectory>("/hand/rh_trajectory_controller/command", , 1, true);

    std::string group_name = "right_hand";
    moveit::planning_interface::MoveGroupInterface mgi(group_name);
    std::string base_frame = mgi.getPoseReferenceFrame();

    robot_model_loader::RobotModelLoader rml;
    auto robot_model = rml.getModel();
    auto joint_model_group = robot_model->getJointModelGroup(group_name);
    moveit::core::RobotState robot_state(robot_model);
    planning_scene::PlanningScene planning_scene(robot_model);

    // get the transform between rh_wrist (csv data are based in rh_wrist) and world
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2::Stamped <tf2::Transform> base_rhWrist_tfstamped;
    geometry_msgs::TransformStamped tfGeom;
    try {
        tfGeom = tfBuffer.lookupTransform(base_frame, "rh_wrist",
                                          ros::Time(0), ros::Duration(5.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    tf2::convert(tfGeom, base_rhWrist_tfstamped);

    // definition of the timeout of the IK solver
    double timeout = 0.02;
    // goal links
    std::vector <std::string> MapPositionlinks {
        "rh_thtip",
        "rh_fftip",
        "rh_mftip",
        "rh_rftip",
        "rh_lftip",
        "rh_thmiddle",
        "rh_ffmiddle",
        "rh_mfmiddle",
        "rh_rfmiddle",
        "rh_lfmiddle"};
    std::vector <std::string> MapDirectionlinks{
        "rh_thproximal",
        "rh_ffproximal",
        "rh_mfproximal",
        "rh_rfproximal",
        "rh_lfproximal",
        "rh_thmiddle"};
    std::vector<float> MapPositionweights{1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 0.2, 0.2};
    std::vector<float> MapDirectionweights{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    // definition of the hand base frame with respect to the camera frame
    Eigen::Matrix4f cam_world_pos(4, 4);

    std::ifstream mapfile(mapfilename);
    std::string line, items;
    std::string item;
    while (std::getline(mapfile, line) && ros::ok()) {
        ros::Time begin = ros::Time::now();
        // track goals using bio ik
        bio_ik::BioIKKinematicsQueryOptions ik_options;
        ik_options.replace = true;
        ik_options.fixed_joints = {"rh_WRJ1", "rh_WRJ2"};
        ik_options.return_approximate_solution = true;

        std::istringstream myline(line);
        std::vector<double> csvItem;
        int n = 0;
        while (std::getline(myline, items, ',')) {
            n++;
            if (n == 1) {
                item = items;
                ROS_INFO_STREAM("Image is : " << item);
                continue;
            }
            csvItem.push_back(std::stof(items));
        }
        if (csvItem[50] > 0.9)
        {
            ROS_WARN("Hand is too for way, skip");
            continue;
        }

        for (int j = 0; j < MapPositionlinks.size(); j++) {
            int t = j * 3;
            tf2::Vector3 stamped_in(csvItem[t], csvItem[t + 1], csvItem[t + 2]);
            tf2::Vector3 Mapposition = base_rhWrist_tfstamped * stamped_in;
            if (j==4 || j==9)
                Mapposition.setZ(Mapposition.z() + 0.06);
            else
                Mapposition.setZ(Mapposition.z() + 0.05);

//            std::cout << "stamped_in : "  << csvItem[t]<< " " << csvItem[t+1]<< " " << csvItem[t+2]<< std::endl;
//            std::cout << "Mapposition : "  << Mapposition.x() << " " << Mapposition.y()<< " " << Mapposition.z()<< std::endl;

            ik_options.goals.emplace_back(
                    new bio_ik::PositionGoal(MapPositionlinks[j], Mapposition, MapPositionweights[j]));
            if (show_hand_inRviz)
            {
                visualization_msgs::Marker point;
                point.ns = std::to_string(j);
                point.header.frame_id = "/world";
                point.type = visualization_msgs::Marker::SPHERE;
                point.action = visualization_msgs::Marker::ADD;
                point.color.g = 1.0f;
                point.color.a = 1.0;
                point.pose.orientation.w = 1.0;
                point.scale.x = 0.01;
                point.scale.y = 0.01;
                point.scale.z = 0.01;
                point.pose.position.x = Mapposition.x();
                point.pose.position.y = Mapposition.y();
                point.pose.position.z = Mapposition.z();
                hand_points.markers.push_back(point);
            }
        }

        for (int j = 0; j < MapDirectionlinks.size(); j++) {
            int t = 30 + j * 3;
            tf2::Vector3 Mapdirection(csvItem[t], csvItem[t + 1], csvItem[t + 2]);
            ik_options.goals.emplace_back(
                    new bio_ik::DirectionGoal(MapDirectionlinks[j], tf2::Vector3(0, 0, 1), Mapdirection.normalized(),
                                              MapDirectionweights[j]));
        }

        // set ik solver
        bool found_ik = robot_state.setFromIK(
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
                            bool collision = (bool) (planning_scene.isStateColliding(
                                    *state, group->getName(), true));
                            if (collision) {
                                ROS_ERROR_STREAM("collision, solution rejected");
                            }
                            return !collision;
                        }),
                ik_options
        );

        std::vector<double> joint_values;
        if (found_ik) {
            // get the solution joints
            robot_state.copyJointGroupPositions(joint_model_group, joint_values);
            if (show_hand_inRviz)
            {
                marker_pub.publish(hand_points);
                // move to the solution position
                mgi.setJointValueTarget(joint_values);
                mgi.move();
            }

            trajectory_msgs::JointTrajectory hand_joints_goal;
            hand_joints_goal.points.emplace_back();
            hand_joints_goal.joint_names = joint_model_group->getVariableNames();

            for (auto &joint : joint_values)
                hand_joints_goal.points.back().positions.emplace_back(joint);

            hand_joints_goal.points.back().time_from_start = ros::Duration(1.0 / frequency);
            hand_pub.publish(hand_joints_goal);

            ros::Duration dur = ros::Time::now() - begin;
            std::cout << "Running time is " << dur << std::endl;
        }
        else
            std::cout << "Did not find IK solution" << std::endl;

        for (int j = 0; j < ik_options.goals.size(); j++)
            ik_options.goals[j].reset();
    }
    joints_file.close();
}
