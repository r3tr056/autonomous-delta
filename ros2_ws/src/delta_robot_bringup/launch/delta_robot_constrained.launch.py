#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time if true'
    )

    # Robot description
    desc_share = FindPackageShare('delta_robot_description').find('delta_robot_description')
    urdf_path = os.path.join(desc_share, 'urdf', 'delta_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # GUI publishes to joint_states_raw
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[('joint_states', 'joint_states_raw')]
    )

    # Constraints solver
    constraints = Node(
        package='delta_robot_control',
        executable='delta_constraints',
        name='delta_constraints',
        output='screen'
    )

    # State publisher consumes constrained joint_states
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        jsp,
        constraints,
        rsp,
    ])
