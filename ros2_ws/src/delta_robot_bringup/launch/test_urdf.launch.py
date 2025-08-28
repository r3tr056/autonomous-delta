#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    # Get package directory
    desc_share = FindPackageShare('delta_robot_description')
    bringup_share = FindPackageShare('delta_robot_bringup')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='true',
        description='Start joint state publisher GUI'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz for visualization'
    )

    # Robot description
    urdf_path = PathJoinSubstitution([desc_share, 'urdf', 'delta_robot.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # RViz config
    rviz_config = PathJoinSubstitution([
        desc_share, 'config', 'delta_robot.rviz'
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Joint State Publisher (for manual control)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'source_list': ['joint_states_manual']
        }]
    )

    # Joint State Publisher GUI (for manual joint control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('joint_states', 'joint_states_manual')
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Static transform publisher for base_link to world (if needed)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # URDF validation node (custom debugging)
    urdf_validator = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='urdf_validator',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 10.0
        }],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_gui_arg,
        use_rviz_arg,

        # Core nodes
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        static_tf,

        # Visualization
        rviz,
    ])
