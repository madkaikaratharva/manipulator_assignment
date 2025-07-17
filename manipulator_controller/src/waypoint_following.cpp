#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"

#include <vector>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);
    auto move_group_node = rclcpp::Node::make_shared("waypoint_following_node", node_options);

    // Spin the node
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "arm";

    // Define Velocity and Acceleration limits
    std::unordered_map<std::string, double> velocity_limits = {
    {"vertical_slider_joint", 0.5},
    {"shoulder_pan_joint", 0.9},
    {"shoulder_lift_joint", 0.9},
    {"elbow_joint", 0.9},
    {"wrist_1_joint", 1.0},
    {"wrist_2_joint", 1.0},
    {"wrist_3_joint", 1.0}
    };

    std::unordered_map<std::string, double> acceleration_limits = {
    {"vertical_slider_joint", 2.5},
    {"shoulder_pan_joint", 2.5},
    {"shoulder_lift_joint", 2.5},
    {"elbow_joint", 2.5},
    {"wrist_1_joint", 2.5},
    {"wrist_2_joint", 2.5},
    {"wrist_3_joint", 2.5}
    };


    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //const moveit::core::JointModelGroup* joint_model_group =
    //  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    RCLCPP_INFO(move_group_node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(move_group_node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Define Waypoints
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    
    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose pose = start_pose;

    // Move Up
    pose.position.z += 0.05;
    waypoints.push_back(pose);

    // Move backward
    pose.position.x += -0.15;
    waypoints.push_back(pose);

    // Move Forward
    pose.position.x += 0.10;
    waypoints.push_back(pose);

    // Move sideways
    pose.position.y += 0.1;
    waypoints.push_back(pose);

    // Move backward
    pose.position.x += -0.05;
    waypoints.push_back(pose);

    // Move sideways
    pose.position.y += -0.15;
    waypoints.push_back(pose);

    // Move Forward
    pose.position.x += 0.15;
    waypoints.push_back(pose);

    // Move sideways
    pose.position.y += -0.05;
    waypoints.push_back(pose);

    // Plan Cartesian Path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);


    // Time Parametrization and Optimization
    robot_trajectory::RobotTrajectory rt(
        move_group.getRobotModel(),
        move_group.getName()
    );
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    //trajectory_processing::IterativeParabolicTimeParameterization totg;
    totg.computeTimeStamps(rt, velocity_limits, acceleration_limits);

    // Convert Message back
    moveit_msgs::msg::RobotTrajectory timed_trajectory;
    rt.getRobotTrajectoryMsg(timed_trajectory);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = timed_trajectory;

    //bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    moveit::core::MoveItErrorCode result = move_group.execute(my_plan);

    if(result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(move_group_node->get_logger(), "Plan successful");
        //move_group.execute(my_plan);
    }
    else
    {
        RCLCPP_INFO(move_group_node->get_logger(), "Plan unsuccessful");   
    }

    rclcpp::shutdown();
    return 0;

}