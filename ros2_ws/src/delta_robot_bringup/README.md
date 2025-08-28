# Delta Robot Bringup

This package provides launch files and configuration for the autonomous delta robot system, supporting both simulation and real hardware deployment.

## Overview

The delta robot system consists of:
- **Hardware**: ESP32 firmware controlling stepper motors, pump, and chassis
- **Vision**: Camera + YOLO detection for weed identification  
- **Control**: Inverse kinematics controller for autonomous weed spraying
- **Teleop**: PS4 controller for manual operation
- **Simulation**: URDF model with physics simulation for testing

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   PS4 Controller   │    │   Camera + YOLO    │    │   Delta Robot   │
│    (Teleop)     │    │   (Detection)    │    │   Controller    │
└─────────┬───────┘    └─────────┬────────┘    └─────────┬───────┘
          │                      │                       │
          └──────────────────────┼───────────────────────┘
                                 │
          ┌─────────────────────────────────────────────────┐
          │                                                 │
          ▼                                                 ▼
  ┌─────────────────┐                           ┌─────────────────┐
  │  ESP32 Firmware │                           │  Simulation     │
  │   (Hardware)    │                           │   (URDF)        │
  └─────────────────┘                           └─────────────────┘
```

## Quick Start

### Prerequisites

```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install additional packages
sudo apt install ros-humble-joy ros-humble-micro-ros-agent
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install python3-pip
pip3 install ultralytics opencv-python
```

### Build the workspace

```bash
cd autonomous-delta/ros2_ws
colcon build
source install/setup.bash
```

## Simulation Mode

### Basic Simulation (Visualization Only)

Launch the robot simulation with RViz:

```bash
ros2 launch delta_robot_bringup simulation.launch.py
```

Parameters:
- `use_rviz:=true` - Start RViz visualization (default: true)
- `use_controller:=true` - Start autonomous controller (default: true)  
- `use_joint_gui:=false` - Start joint state publisher GUI (default: false)

### Simulation with Teleop (Recommended)

Launch simulation with PS4 controller support:

```bash
# Connect PS4 controller via USB/Bluetooth first
ros2 launch delta_robot_bringup simulation_teleop.launch.py
```

Parameters:
- `joy_device_id:=0` - Joystick device index (default: 0)
- `use_mock_camera:=true` - Enable mock camera/detection for testing
- `use_controller:=true` - Enable autonomous mode alongside teleop

### Manual Joint Control

For direct joint manipulation:

```bash
ros2 launch delta_robot_bringup simulation.launch.py use_joint_gui:=true use_controller:=false
```

## Hardware Mode

### Hardware with Teleop (Typical Usage)

```bash
# Ensure ESP32 is connected via USB
ros2 launch delta_robot_bringup system.launch.py simulation_mode:=false
```

Parameters:
- `serial_dev:=/dev/ttyUSB0` - ESP32 serial device
- `start_agent:=true` - Start micro-ROS agent  
- `joy_device_id:=0` - PS4 controller device

### Hardware Only (No Teleop)

```bash
ros2 launch delta_robot_bringup controller.launch.py
```

## Control Modes

### PS4 Controller Layout

```
          [△] Enable Motors              [L1] Deadman Switch (HOLD)
    [□]       [○] Disable Motors         [L2] Jog Z Down  
          [✕] Soft Zero                  [R1] Spray (Manual Mode)
                                         [R2] Jog Z Up

    Left Stick              Right Stick
    ┌─────────┐            ┌─────────────┐
    │    ↑    │ +Y (Fwd)   │      ↑      │ +Z (Up)
    │  ←   → │ ±X         │   ←     →   │ Chassis ±
    │    ↓    │ -Y (Back)  │      ↓      │ -Z (Down)
    └─────────┘            └─────────────┘

[OPTIONS] Toggle Manual/Auto Mode
```

**Controls:**
- **Hold L1** to enable movement (deadman switch)
- **Left Stick**: Move end effector X/Y in manual mode
- **Right Stick**: Move end effector Z (vertical) / Chassis horizontal
- **R1**: Spray pump (manual mode only)
- **Options**: Toggle between Manual and Autonomous modes
- **Triangle**: Enable motor drivers
- **Circle**: Disable motor drivers  
- **Cross**: Soft zero (return to origin)

### Autonomous Mode

In autonomous mode, the system:
1. Processes camera feed for weed detection
2. Plans path to detected weeds
3. Moves to hover position above target
4. Descends to spray height
5. Activates pump for configured spray time
6. Retracts and moves to next target

### Manual Mode

In manual mode:
- Direct joystick control of end effector
- Hold L1 + move sticks to control robot
- Hold R1 to activate spray pump
- Real-time position control

## Launch File Reference

### system.launch.py (Main Entry Point)

```bash
ros2 launch delta_robot_bringup system.launch.py [options]
```

**Key Parameters:**
- `simulation_mode:=true/false` - Choose sim or hardware
- `joy_device_id:=0` - PS4 controller device
- `use_feedback:=true` - Use ArUco marker feedback
- `chassis_auto_speed:=0.2` - Chassis creep speed in auto mode

### simulation_teleop.launch.py (Full Simulation)

```bash
ros2 launch delta_robot_bringup simulation_teleop.launch.py [options]
```

**Simulation Parameters:**
- `use_rviz:=true` - Start RViz
- `use_mock_camera:=false` - Enable fake camera/detection
- `use_controller:=true` - Enable autonomous controller

### controller.launch.py (Hardware Only)

```bash
ros2 launch delta_robot_bringup controller.launch.py [options]
```

**Hardware Parameters:**
- `start_agent:=true` - Start micro-ROS agent
- `serial_dev:=/dev/ttyUSB0` - ESP32 serial port

### hardware.launch.py (Sensors Only)

```bash
ros2 launch delta_robot_bringup hardware.launch.py [options]
```

Starts camera, detector, ArUco tracker, and teleop nodes only.

## Topics Reference

### Control Topics
- `/delta_joint_target` - Joint angle commands (Float32MultiArray)
- `/delta_joint_position` - Current joint angles (Float32MultiArray) 
- `/pump_cmd` - Pump duty cycle (Float32)
- `/chassis_cmd` - Chassis movement command (Float32)
- `/delta_command` - Firmware commands (String)

### Teleop Topics  
- `/delta_teleop/manual` - Manual mode flag (Bool)
- `/delta_teleop/twist` - End effector velocity (Twist)
- `/delta_teleop/spray` - Spray command (Bool)
- `/delta_teleop/enable` - Motor enable (Bool)
- `/delta_teleop/soft_zero` - Soft zero command (Bool)

### Vision Topics
- `/camera/image_raw` - Camera feed (Image)
- `/detections` - Detected weeds (Detection2DArray) 
- `/ee_feedback` - End effector position feedback (PointStamped)

## Troubleshooting

### PS4 Controller Issues

```bash
# List available joystick devices
ros2 run joy joy_enumerate_devices

# Test controller input
ros2 topic echo /joy
```

### ESP32 Connection Issues

```bash
# Check serial ports
ls /dev/ttyUSB* /dev/ttyACM*

# Test micro-ROS agent manually
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### Simulation Not Moving

1. Check if motors are enabled: `Triangle` button or `ros2 topic pub /delta_teleop/enable std_msgs/Bool "data: true"`
2. Verify deadman switch: Hold `L1` while moving sticks
3. Check joint limits in RViz

### Camera/Detection Issues

```bash
# Test camera feed
ros2 topic hz /camera/image_raw
ros2 run rqt_image_view rqt_image_view

# Check detection output  
ros2 topic echo /detections --once
```

## Development

### Adding New Features

1. **Simulation**: Modify `delta_robot_sim.py` 
2. **Control**: Modify `delta_ik.py`
3. **Hardware**: Update ESP32 firmware in `firmware/delta-fw/main/main.c`

### Testing Workflow

1. Test in simulation first: `simulation_mode:=true`
2. Validate with mock camera: `use_mock_camera:=true`  
3. Deploy to hardware: `simulation_mode:=false`

### Custom Robot Parameters

Edit robot dimensions in:
- URDF: `delta_robot_description/urdf/delta_robot.urdf.xacro`
- IK: `delta_ik.py` constructor parameters
- Firmware: `main.c` defines

## Safety Notes

⚠️ **IMPORTANT SAFETY GUIDELINES**

1. **Always test in simulation first**
2. **Keep emergency stop accessible**
3. **Ensure workspace is clear before autonomous operation** 
4. **Use deadman switch (L1) in manual mode**
5. **Check joint limits match physical robot**
6. **Verify spray system is safe before operation**

## Support

- Check launch file parameters match your setup
- Verify all dependencies are installed
- Test individual nodes separately if issues persist
- Monitor ROS logs for error messages: `ros2 topic echo /rosout`
