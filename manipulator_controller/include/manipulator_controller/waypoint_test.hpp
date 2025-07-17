#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class WaypointTest : public rclcpp::Node
{
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Velocities and Accelerations

    // Timer Callback
    void timercallback();

public:
    WaypointTest();

};