# delta_robot_bringup/launch/controller.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Whether to start micro-ROS agent and which serial device to use
    start_agent = DeclareLaunchArgument(
        'start_agent', default_value='true',
        description='Start micro-ROS agent for ESP32 over serial.'
    )
    serial_dev = DeclareLaunchArgument(
        'serial_dev', default_value='/dev/ttyUSB0',
        description='Serial device for micro-ROS agent.'
    )

    # Controller parameters
    use_feedback = DeclareLaunchArgument(
        'use_feedback', default_value='true',
        description='Use EE ArUco feedback for fine alignment.'
    )
    chassis_auto_speed = DeclareLaunchArgument(
        'chassis_auto_speed', default_value='0.2',
        description='Signed [-1..1] chassis creep speed command in auto.'
    )

    # Delta controller
    controller = Node(
        package='delta_robot_control',
        executable='delta_ik.py',
        name='delta_robot_controller',
        output='screen',
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
        }],
    )

    # Optional micro-ROS agent (serial)
    agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', LaunchConfiguration('serial_dev')],
        condition=IfCondition(LaunchConfiguration('start_agent')),
    )

    return LaunchDescription([
        start_agent, serial_dev, use_feedback, chassis_auto_speed,
        controller, agent
    ])
