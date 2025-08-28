#!/usr/bin/env python3

import argparse
import subprocess
import sys
import os
from pathlib import Path


def run_command(cmd, check=True):
    """Run a shell command"""
    print(f"Running: {' '.join(cmd)}")
    try:
        result = subprocess.run(cmd, check=check)
        return result.returncode == 0
    except subprocess.CalledProcessError as e:
        print(f"Command failed with exit code {e.returncode}")
        return False
    except KeyboardInterrupt:
        print("\nStopped by user")
        return False


def check_ros2_sourced():
    """Check if ROS2 is properly sourced"""
    if 'ROS_DISTRO' not in os.environ:
        print("❌ ROS2 not sourced. Please run:")
        print("source /opt/ros/humble/setup.bash")
        print("source ~/ros2_ws/install/setup.bash")
        return False
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Delta Robot System Launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s sim                          # Basic simulation with RViz
  %(prog)s sim --teleop                 # Simulation with PS4 controller
  %(prog)s sim --teleop --mock-camera   # Full simulation with fake detection
  %(prog)s hw                           # Hardware mode with teleop
  %(prog)s hw --no-teleop               # Hardware autonomous only
        """)

    # Mode selection
    parser.add_argument('mode', choices=['sim', 'hardware', 'hw'],
                       help='Run in simulation (sim) or hardware (hw/hardware) mode')

    # Common options
    parser.add_argument('--no-rviz', action='store_true',
                       help='Disable RViz (simulation only)')
    parser.add_argument('--no-controller', action='store_true',
                       help='Disable autonomous controller')
    parser.add_argument('--teleop', action='store_true',
                       help='Enable PS4 teleop (simulation only - always on for hardware)')
    parser.add_argument('--mock-camera', action='store_true',
                       help='Enable mock camera and detection (simulation only)')

    # Hardware-specific options
    parser.add_argument('--serial-dev', default='/dev/ttyUSB0',
                       help='Serial device for ESP32 (default: /dev/ttyUSB0)')
    parser.add_argument('--no-agent', action='store_true',
                       help='Disable micro-ROS agent (hardware mode only)')

    # Controller options
    parser.add_argument('--no-feedback', action='store_true',
                       help='Disable ArUco end effector feedback')
    parser.add_argument('--chassis-speed', type=float, default=0.2,
                       help='Chassis auto speed -1..1 (default: 0.2)')

    # Teleop options
    parser.add_argument('--joy-device', type=int, default=0,
                       help='Joystick device ID (default: 0)')
    parser.add_argument('--joy-deadzone', type=float, default=0.1,
                       help='Joystick deadzone (default: 0.1)')

    # Special modes
    parser.add_argument('--joint-gui', action='store_true',
                       help='Manual joint control with GUI (simulation only)')
    parser.add_argument('--display-only', action='store_true',
                       help='Just robot display, no control (simulation only)')

    args = parser.parse_args()

    # Validate environment
    if not check_ros2_sourced():
        sys.exit(1)

    # Normalize mode
    is_simulation = args.mode == 'sim'
    is_hardware = args.mode in ['hw', 'hardware']

    print("🤖 Delta Robot System Launcher")
    print("=" * 40)
    print(f"Mode: {'Simulation' if is_simulation else 'Hardware'}")

    # Build launch command
    if args.display_only and is_simulation:
        # Just robot description and RViz
        launch_file = 'delta_robot_description display.launch.py'
        launch_args = []
    elif args.joint_gui and is_simulation:
        # Manual joint control
        launch_file = 'delta_robot_bringup simulation.launch.py'
        launch_args = [
            f'use_joint_gui:=true',
            f'use_controller:=false',
            f'use_rviz:={not args.no_rviz}'
        ]
    elif is_simulation and not args.teleop:
        # Basic simulation
        launch_file = 'delta_robot_bringup simulation.launch.py'
        launch_args = [
            f'use_rviz:={not args.no_rviz}',
            f'use_controller:={not args.no_controller}',
            f'use_joint_gui:=false'
        ]
    elif is_simulation and args.teleop:
        # Full simulation with teleop
        launch_file = 'delta_robot_bringup simulation_teleop.launch.py'
        launch_args = [
            f'use_rviz:={not args.no_rviz}',
            f'use_controller:={not args.no_controller}',
            f'use_mock_camera:={args.mock_camera}',
            f'joy_device_id:={args.joy_device}',
            f'joy_deadzone:={args.joy_deadzone}',
            f'use_feedback:={not args.no_feedback}',
            f'chassis_auto_speed:={args.chassis_speed}'
        ]
    elif is_hardware:
        # Hardware mode (always uses system.launch.py)
        launch_file = 'delta_robot_bringup system.launch.py'
        launch_args = [
            f'simulation_mode:=false',
            f'start_agent:={not args.no_agent}',
            f'serial_dev:={args.serial_dev}',
            f'joy_device_id:={args.joy_device}',
            f'joy_deadzone:={args.joy_deadzone}',
            f'use_feedback:={not args.no_feedback}',
            f'chassis_auto_speed:={args.chassis_speed}',
            f'use_controller:={not args.no_controller}'
        ]
    else:
        print("❌ Invalid configuration")
        sys.exit(1)

    # Print configuration
    print(f"Launch file: {launch_file}")
    if launch_args:
        print("Parameters:")
        for arg in launch_args:
            print(f"  {arg}")

    # Hardware mode warnings
    if is_hardware:
        print("\n⚠️  HARDWARE MODE WARNINGS:")
        print("  - Ensure ESP32 is connected and powered")
        print("  - Check robot workspace is clear")
        print("  - Have emergency stop ready")
        print("  - PS4 controller should be connected")

        response = input("\nContinue? [y/N]: ").strip().lower()
        if response != 'y':
            print("Cancelled by user")
            sys.exit(0)

    print("\n🚀 Starting delta robot system...")
    print("Press Ctrl+C to stop")
    print("-" * 40)

    # Execute launch command
    cmd = ['ros2', 'launch'] + launch_file.split() + launch_args

    success = run_command(cmd, check=False)

    if success:
        print("\n✅ Delta robot system stopped normally")
    else:
        print("\n❌ Delta robot system stopped with errors")
        sys.exit(1)


if __name__ == '__main__':
    main()
