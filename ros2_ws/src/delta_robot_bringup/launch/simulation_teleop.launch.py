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

    use_mock_camera_arg = DeclareLaunchArgument(
        'use_mock_camera', default_value='false',
        description='Start mock camera and detection nodes for testing'
    )

    # PS4 Controller arguments
    joy_device_id_arg = DeclareLaunchArgument(
        'joy_device_id', default_value='0',
        description='Index of joystick (ros2 run joy joy_enumerate_devices to list)'
    )
    joy_deadzone_arg = DeclareLaunchArgument(
        'joy_deadzone', default_value='0.1',
        description='Deadzone for joystick axes'
    )
    joy_autorepeat_arg = DeclareLaunchArgument(
        'joy_autorepeat_rate', default_value='20.0',
        description='Autorepeat rate for joy messages when sticks are held'
    )

    # Controller parameters
    use_feedback_arg = DeclareLaunchArgument(
        'use_feedback', default_value='false',
        description='Use EE ArUco feedback (disabled in simulation by default)'
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

    # ============================================================================
    # CORE SIMULATION NODES
    # ============================================================================

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

    # ============================================================================
    # TELEOP NODES (PS4 Controller)
    # ============================================================================

    # ROS 2 joy node (Humble joy package)
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('joy_device_id'),
            'deadzone': LaunchConfiguration('joy_deadzone'),
            'autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
            'sticky_buttons': False,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # PS4 teleop node (publishes delta_teleop/* and chassis_cmd)
    teleop = Node(
        package='delta_robot_hardware',
        executable='teleop_node.py',
        name='ps4_teleop',
        output='screen',
        parameters=[{
            'deadzone': LaunchConfiguration('joy_deadzone'),
            'max_xy_cm_s': 8.0,
            'max_z_cm_s': 6.0,
            'max_chassis_cmd': 1.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # ============================================================================
    # CONTROLLER NODE (Autonomous Mode)
    # ============================================================================

    # Delta Robot Controller
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

    # ============================================================================
    # MOCK HARDWARE NODES (Optional - for testing)
    # ============================================================================

    # Mock camera node (publishes test image)
    mock_camera = Node(
        package='delta_robot_hardware',
        executable='camera_node.py',
        name='mock_camera_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mock_camera')),
        parameters=[{
            'width': 640,
            'height': 480,
            'framerate': 30,
            'topic': 'camera/image_raw',
            'stabilize': False,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Mock detector node (for autonomous testing)
    mock_detector = Node(
        package='delta_robot_hardware',
        executable='detector_node.py',
        name='mock_yolo_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mock_camera')),
        parameters=[{
            'image_topic': 'camera/image_raw',
            'detections_topic': 'detections',
            'annotated_topic': 'camera/image_annotated',
            'conf': 0.5,
            'iou': 0.5,
            'imgsz': 320,
            'device': 'cpu',
            'pixels_per_cm': 5.0,
            'weed_class_id': 1,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # ============================================================================
    # VISUALIZATION
    # ============================================================================

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

    # Static transform for camera frame (if using mock camera)
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'camera_frame'],
        condition=IfCondition(LaunchConfiguration('use_mock_camera')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_rviz_arg,
        use_controller_arg,
        use_mock_camera_arg,
        joy_device_id_arg,
        joy_deadzone_arg,
        joy_autorepeat_arg,
        use_feedback_arg,
        chassis_auto_speed_arg,

        # Core simulation nodes
        robot_state_publisher,
        delta_sim,

        # Teleop nodes
        joy,
        teleop,

        # Controller (autonomous mode)
        delta_controller,

        # Mock hardware (optional)
        mock_camera,
        mock_detector,
        static_tf_camera,

        # Visualization
        rviz,
    ])
