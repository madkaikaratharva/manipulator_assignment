from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    moveit_config = (
        MoveItConfigsBuilder("arm_manipulator", package_name="manipulator_moveit")
        .to_moveit_configs()
    )

    # Inject 'use_sim_time' into robot_description node parameters
    moveit_config.robot_description["use_sim_time"] = use_sim_time

    return generate_move_group_launch(moveit_config)
