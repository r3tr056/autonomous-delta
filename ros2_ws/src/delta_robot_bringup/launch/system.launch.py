# delta_robot_bringup/launch/system.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_share = get_package_share_directory('delta_robot_bringup')

    # Mode selection
    simulation_mode = DeclareLaunchArgument(
        'simulation_mode', default_value='false',
        description='Run in simulation mode (true) or hardware mode (false)'
    )

    # Shared arguments
    joy_device_id = DeclareLaunchArgument('joy_device_id', default_value='0')
    joy_deadzone = DeclareLaunchArgument('joy_deadzone', default_value='0.1')
    joy_autorepeat_rate = DeclareLaunchArgument('joy_autorepeat_rate', default_value='20.0')

    # Hardware-specific arguments
    start_agent = DeclareLaunchArgument('start_agent', default_value='true')
    serial_dev = DeclareLaunchArgument('serial_dev', default_value='/dev/ttyUSB0')

    # Simulation-specific arguments
    use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')
    use_mock_camera = DeclareLaunchArgument('use_mock_camera', default_value='false')

    # Shared controller arguments
    use_feedback = DeclareLaunchArgument('use_feedback', default_value='true')
    chassis_auto_speed = DeclareLaunchArgument('chassis_auto_speed', default_value='0.2')
    use_controller = DeclareLaunchArgument('use_controller', default_value='true')

    # SIMULATION MODE: Include simulation teleop launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'simulation_teleop.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('simulation_mode')),
        launch_arguments={
            'joy_device_id': LaunchConfiguration('joy_device_id'),
            'joy_deadzone': LaunchConfiguration('joy_deadzone'),
            'joy_autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_controller': LaunchConfiguration('use_controller'),
            'use_mock_camera': LaunchConfiguration('use_mock_camera'),
            'use_feedback': LaunchConfiguration('use_feedback'),
            'chassis_auto_speed': LaunchConfiguration('chassis_auto_speed'),
        }.items()
    )

    # HARDWARE MODE: Include hardware launch
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'hardware.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('simulation_mode')),
        launch_arguments={
            'joy_device_id': LaunchConfiguration('joy_device_id'),
            'joy_deadzone': LaunchConfiguration('joy_deadzone'),
            'joy_autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
        }.items()
    )

    # HARDWARE MODE: Include controller launch
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'controller.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('simulation_mode')),
        launch_arguments={
            'start_agent': LaunchConfiguration('start_agent'),
            'serial_dev': LaunchConfiguration('serial_dev'),
            'use_feedback': LaunchConfiguration('use_feedback'),
            'chassis_auto_speed': LaunchConfiguration('chassis_auto_speed'),
        }.items()
    )

    return LaunchDescription([
        # Mode selection
        simulation_mode,

        # Shared arguments
        joy_device_id, joy_deadzone, joy_autorepeat_rate,
        use_feedback, chassis_auto_speed, use_controller,

        # Hardware-specific arguments
        start_agent, serial_dev,

        # Simulation-specific arguments
        use_rviz, use_mock_camera,

        # Launch files (conditional based on mode)
        simulation_launch,
        hardware,
        controller
    ])
