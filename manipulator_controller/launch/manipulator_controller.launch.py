import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen"
    )

    simple_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joint_trajectory_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        output="screen"
    )

    end_effector_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["end_effector_trajectory_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        output="screen"
    )

    forward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["vertical_effort_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        output="screen"
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        simple_controller_spawner,
    ])