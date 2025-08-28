#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    # Package directories
    bringup_share = FindPackageShare('delta_robot_bringup')
    desc_share = FindPackageShare('delta_robot_description')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz for visualization'
    )

    use_controller_arg = DeclareLaunchArgument(
        'use_controller', default_value='true',
        description='Start delta robot controller for autonomous mode'
    )

    use_joint_gui_arg = DeclareLaunchArgument(
        'use_joint_gui', default_value='false',
        description='Start joint state publisher GUI for manual joint control'
    )

    # Controller parameters (same as controller.launch.py)
    use_feedback_arg = DeclareLaunchArgument(
        'use_feedback', default_value='false',
        description='Use EE ArUco feedback (disabled in simulation)'
    )
    chassis_auto_speed_arg = DeclareLaunchArgument(
        'chassis_auto_speed', default_value='0.2',
        description='Signed [-1..1] chassis creep speed in auto mode'
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

    # Delta Robot Simulation Node (replaces ESP32 firmware)
    delta_sim = Node(
        package='delta_robot_control',
        executable='delta_robot_sim',
        name='delta_robot_sim',
        output='screen',
        parameters=[{
            'joint_names': [
                'base_brazo1', 'base_brazo2', 'base_brazo3',
                'codo1_a', 'codo1_b', 'codo2_a', 'codo2_b', 'codo3_a', 'codo3_b',
                'forearm1_ee_x', 'forearm1_ee_y'
            ],
            'max_joint_velocity': 2.0,
            'max_joint_acceleration': 4.0,
            'position_tolerance': 0.01,
            'control_frequency': 100.0,
            'enable_constraints': False,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Delta Robot Controller (autonomous mode)
    delta_controller = Node(
        package='delta_robot_control',
        executable='delta_ik.py',
        name='delta_robot_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_controller')),
        parameters=[{
            'detections_topic': 'detections',
            'joint_target_topic': 'delta_joint_target',
            'joint_state_topic': 'delta_joint_position',
            'ee_feedback_topic': 'ee_feedback',
            'use_feedback': LaunchConfiguration('use_feedback'),
            'spray_time_s': 0.5,
            'spray_duty': 0.8,
            'replan_new_targets': 3,
            'target_stale_s': 5.0,
            'xy_speed_cm_s': 6.0,
            'z_speed_cm_s': 6.0,
            'hover_z_cm': -8.0,
            'spray_z_cm': -12.0,
            'chassis_auto_speed': LaunchConfiguration('chassis_auto_speed'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Joint State Publisher GUI (for manual joint control)
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_gui')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('joint_states', 'joint_states_manual')  # Don't interfere with simulation
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

    # Static transform for simulation (camera/world frame if needed)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'camera_frame'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_rviz_arg,
        use_controller_arg,
        use_joint_gui_arg,
        use_feedback_arg,
        chassis_auto_speed_arg,

        # Core simulation nodes
        robot_state_publisher,
        delta_sim,
        static_tf,

        # Optional nodes
        delta_controller,
        joint_state_gui,
        rviz,
    ])
