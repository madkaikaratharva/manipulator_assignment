#include "manipulator_controller/waypoint_test.hpp"

using namespace std::chrono_literals;


WaypointTest::WaypointTest()
    : Node("simple_waypoint_test")
{
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_joint_trajectory_controller/joint_trajectory", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&WaypointTest::timercallback, this));
}

void WaypointTest::timercallback()
{
    trajectory_msgs::msg::JointTrajectory trajectory;

    trajectory.header.stamp = rclcpp::Time(0);

    // Add Joint Names
    trajectory.joint_names.push_back("vertical_slider_joint");
    trajectory.joint_names.push_back("shoulder_pan_joint");
    trajectory.joint_names.push_back("shoulder_lift_joint");
    trajectory.joint_names.push_back("elbow_joint");
    trajectory.joint_names.push_back("wrist_1_joint");
    trajectory.joint_names.push_back("wrist_2_joint");
    trajectory.joint_names.push_back("wrist_3_joint");

    for(int i=0; i<1; i++)
    {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        //  Push Back Positions
        point.positions.push_back(1.0);
        point.positions.push_back(1.57);
        point.positions.push_back(-1.57);
        point.positions.push_back(1.57);
        point.positions.push_back(1.57);
        point.positions.push_back(1.57);
        point.positions.push_back(0.0);

        //  Push Back velocities
        point.velocities.push_back(0.0);
        point.velocities.push_back(0.0);
        point.velocities.push_back(0.0);
        point.velocities.push_back(0.0);
        point.velocities.push_back(0.0);
        point.velocities.push_back(0.0);
        point.velocities.push_back(0.0);

        point.accelerations = std::vector<double>(7, 0.0);
        
        point.effort.clear();

        point.time_from_start = rclcpp::Duration::from_seconds(5.0);

        // Push Point to Trajectory
        trajectory.points.push_back(point);

    }

    RCLCPP_INFO(get_logger(), "Publishing Trajectory");
    traj_pub_->publish(trajectory);
    RCLCPP_INFO(get_logger(), "Published Trajectory");

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}