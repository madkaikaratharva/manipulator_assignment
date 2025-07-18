#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"

#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"

int main(int argc, char** argv)
{   
    // Initialize ROS interface
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);
    auto move_group_node = rclcpp::Node::make_shared("plan_and_execute_node", node_options);

    // Spin the node
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Declare Planning Group
    static const std::string PLANNING_GROUP = "arm";

    // Define Velocity and Acceleration limits
    std::unordered_map<std::string, double> velocity_limits = {
    {"vertical_slider_joint", 1.0},
    {"shoulder_pan_joint", 3.14},
    {"shoulder_lift_joint", 2.0},
    {"elbow_joint", 3.14},
    {"wrist_1_joint", 3.14},
    {"wrist_2_joint", 3.14},
    {"wrist_3_joint", 3.14}
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

    // Initialize the Move Group Interface
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Set Velocity Scaling and planning time
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);

    // Robot Home Position
    move_group.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RCLCPP_INFO(move_group_node->get_logger(), "Returning to home position");   
        move_group.execute(plan);
        RCLCPP_INFO(move_group_node->get_logger(), "Returned to home position");

    }

    // Planning frame and end effector link
    RCLCPP_INFO(move_group_node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(move_group_node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());


    // Set Goal Poses
    std::vector<geometry_msgs::msg::Pose> goal_poses;

    // Goal Pose 1
    // Get the start pose of the robot
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose pose1 = start_pose;
    pose1.position.x = 0.25;
    pose1.position.y = 0.30;
    goal_poses.push_back(pose1);

    // Define Goal Pose 2
    geometry_msgs::msg::Pose pose2 = start_pose;
    pose2.position.x = -0.25;
    pose2.position.y = -0.30;
    goal_poses.push_back(pose2);

    // Iterate over goal poses
    for (size_t i = 0; i < goal_poses.size(); ++i)
    {   
        // Set Goal
        move_group.setPoseTarget(goal_poses[i]);

        // Plan Trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success)
        {
            // Add Time Parameterization.
            // Using time parameterization between trajectory points to attain trapezoidal velocity profiles
            robot_trajectory::RobotTrajectory rt(
                move_group.getRobotModel(),
                move_group.getName()
            );
        
            rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), plan.trajectory_);
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            totg.computeTimeStamps(rt, velocity_limits, acceleration_limits);
        
            // Convert Message back
            moveit_msgs::msg::RobotTrajectory timed_trajectory;
            rt.getRobotTrajectoryMsg(timed_trajectory);
            plan.trajectory_ = timed_trajectory;

            move_group.execute(plan);
            RCLCPP_INFO(move_group_node->get_logger(), "Executed trajectory");
        }
        else
        {
            RCLCPP_INFO(move_group_node->get_logger(), "Plan unsuccessful");
            rclcpp::shutdown();
            return 0; 
        }

        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    rclcpp::shutdown();
    return 0;

}


