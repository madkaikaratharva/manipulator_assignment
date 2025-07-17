#include "rclcpp/rclcpp.hpp"
#include <string>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"

#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"

#include "geometry_msgs/msg/pose.hpp"


class WaypointFollowing : public rclcpp::Node
{
private:
    double vertical_height_;
    double max_radius_;
    double min_radius_;
    double max_z_;
    double min_z_;
    std::string planning_group_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
public:
    WaypointFollowing(const rclcpp::NodeOptions& options);
    void init();

private:
    void run();
};