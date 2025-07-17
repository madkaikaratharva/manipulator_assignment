#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"

class PlanAndExecute : public rclcpp::Node
{
private:
    // Subscriber to get Goal
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;

    // Publisher to Publish Trajectory
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

    // For Testing
    rclcpp::TimerBase::SharedPtr timer_;

    // Move Group Interface Object
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> wrist_group_;

public:
    PlanAndExecute();

private:
    void goalCallback(const geometry_msgs::msg::Pose& goal);

    void timerCallback();

};