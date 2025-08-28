# delta_robot_bringup/launch/hardware.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments for joystick device and behavior
    joy_device_id = DeclareLaunchArgument(
        'joy_device_id', default_value='0',
        description='Index of joystick (ros2 run joy joy_enumerate_devices to list).'
    )
    joy_deadzone = DeclareLaunchArgument(
        'joy_deadzone', default_value='0.1',
        description='Deadzone for joystick axes.'
    )
    joy_autorepeat = DeclareLaunchArgument(
        'joy_autorepeat_rate', default_value='20.0',
        description='Autorepeat rate for joy messages when sticks are held.'
    )

    # Camera node (RGB stream)
    cam = Node(
        package='delta_robot_hardware',
        executable='camera_node.py',
        name='pi_camera_publisher',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'framerate': 30,
            'topic': 'camera/image_raw',
            'stabilize': False,
        }],
    )

    # YOLO detector + tracker
    det = Node(
        package='delta_robot_hardware',
        executable='detector_node.py',
        name='yolo_detector_uv',
        output='screen',
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
        }],
    )

    # EE ArUco tracker for feedback
    aruco = Node(
        package='delta_robot_hardware',
        executable='ee_aruco_tracker.py',
        name='ee_aruco_tracker',
        output='screen',
        parameters=[{
            'image_topic': 'camera/image_raw',
            'feedback_topic': 'ee_feedback',
            'pixels_per_cm': 5.0,
            'aruco_dict': 'DICT_4X4_50',
            'marker_id': 0,
            'display': False,
        }],
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
            # Axis/button indices are parameterized inside the node
        }],
    )

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
        }],
    )

    return LaunchDescription([
        joy_device_id, joy_deadzone, joy_autorepeat,
        cam, det, aruco, teleop, joy
    ])
