# Autonomous Delta Robot System - Agribot

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-brightgreen.svg)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/Docker-Enabled-blue.svg)](https://www.docker.com/)
[![ESP32](https://img.shields.io/badge/Hardware-ESP32-red.svg)](https://www.espressif.com/en/products/socs/esp32)

A complete ROS 2-based autonomous precision agriculture system featuring a delta robot for targeted weed spraying, with full simulation capabilities and ESP32 firmware integration.

![System Overview](docs/images/delta-robot-overview.png)

## 🚀 Features

### 🤖 **Complete Delta Robot System**
- **Inverse Kinematics Controller**: Closed-form Clavel delta robot kinematics
- **Real-time Motion Control**: Smooth trajectory planning with velocity/acceleration limits
- **Precision End Effector**: Targeted spraying with sub-centimeter accuracy
- **Safety Systems**: Emergency stops, joint limits, and collision avoidance

### 🔧 **Dual Mode Operation**
- **Simulation Mode**: Full physics simulation with 3D visualization
- **Hardware Mode**: Direct ESP32 micro-ROS integration
- **Seamless Switching**: Identical interfaces for development and deployment

### 🎮 **Control Interfaces**
- **Autonomous Mode**: YOLO-based weed detection and automated spraying
- **Manual Teleop**: PS4 controller with intuitive control mapping
- **Mixed Mode**: Manual override capabilities during autonomous operation

### 👁️ **Computer Vision Pipeline**
- **Real-time Detection**: YOLO-based weed identification with object tracking
- **Camera Integration**: Raspberry Pi Camera with image stabilization
- **ArUco Feedback**: End effector position feedback for precision alignment
- **Visualization**: Annotated camera feed with detection overlays

### 🐳 **Containerized Deployment**
- **Docker Integration**: Complete system packaged for easy deployment
- **Cross-platform**: Supports x86_64 (development) and ARM64 (Raspberry Pi)
- **Firmware Tools**: ESP32 build and flash capabilities in containers

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   PS4 Controller   │    │   Vision System    │    │   Delta Robot   │
│    (Teleop)     │    │  (YOLO + ArUco)  │    │   Controller    │
└─────────┬───────┘    └─────────┬────────┘    └─────────┬───────┘
          │                      │                       │
          └──────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────────────┐
                    │    ROS 2 Middleware     │
                    │   (Topics/Services)     │
                    └─────────────┬───────────┘
                                 │
          ┌──────────────────────┼──────────────────────┐
          │                      │                      │
          ▼                      ▼                      ▼
  ┌─────────────┐    ┌─────────────────┐    ┌─────────────────┐
  │ Simulation  │    │  ESP32 Firmware │    │  RViz/GUI      │
  │   Node      │    │  (micro-ROS)    │    │ Visualization   │
  └─────────────┘    └─────────────────┘    └─────────────────┘
```

### 📦 **Package Structure**

- **`delta_robot_control`**: Core IK controller and simulation node
- **`delta_robot_hardware`**: Camera, detection, ArUco, and teleop nodes  
- **`delta_robot_description`**: URDF models and visualization
- **`delta_robot_bringup`**: Launch files and system configuration
- **`firmware/delta-fw`**: ESP32 micro-ROS firmware

## 🛠️ Requirements

### **Hardware**
- **Development Machine**: x86_64 or ARM64 with 4GB+ RAM
- **Raspberry Pi 4**: For field deployment (4GB+ recommended)
- **ESP32**: Main controller with micro-ROS firmware
- **Delta Robot**: 3-DOF parallel mechanism with stepper motors
- **Camera**: Raspberry Pi Camera or USB webcam
- **PS4 Controller**: For manual control (USB or Bluetooth)

### **Software**
- **Docker**: Version 20.0+ with Compose plugin
- **ROS 2**: Jazzy Jalopy (automatically installed in containers)
- **Python**: 3.10+ with NumPy, OpenCV, ultralytics
- **X11**: For GUI applications (RViz, joint state publisher)

## ⚡ Quick Start

### 1. **Clone and Setup**
```bash
git clone https://github.com/r3tr056/autonomous-delta.git
cd autonomous-delta
chmod +x deploy.sh install_dependencies.sh
```

### 2. **Deploy with Docker (Recommended)**
```bash
# Build system
./deploy.sh build

# Start simulation
./deploy.sh sim

# In another terminal, run tests
./deploy.sh test
```

### 3. **Native Installation (Optional)**
```bash
# Install dependencies
./install_dependencies.sh

# Build ROS workspace
source setup_workspace.sh
cd ros2_ws && colcon build
source install/setup.bash

# Launch simulation
ros2 launch delta_robot_bringup simulation_teleop.launch.py
```

## 🐳 Docker Deployment

### **Quick Commands**
```bash
# Build all images
./deploy.sh build

# Simulation mode
./deploy.sh sim

# Hardware mode (with ESP32)
./deploy.sh hardware

# Flash ESP32 firmware
./deploy.sh firmware flash

# Monitor ESP32 output
./deploy.sh firmware monitor

# Run system tests
./deploy.sh test

# Show system status
./deploy.sh status

# Clean everything
./deploy.sh clean
```

### **Docker Compose**
```bash
# Start main service
docker-compose up delta-robot-ros

# Development mode
docker-compose --profile dev up delta-robot-dev

# Firmware operations
docker-compose --profile firmware run delta-robot-firmware flash

# Access running container
docker-compose exec delta-robot-ros bash
```

## 🎮 Control Guide

### **PS4 Controller Layout**
```
          [△] Enable Motors              [L1] HOLD for Movement
    [□]       [○] Disable Motors         [L2] (Reserved)
          [✕] Soft Zero                  [R1] Spray Pump  
                                         [R2] (Reserved)

    Left Stick              Right Stick
    ┌─────────┐            ┌─────────────┐
    │    ↑    │ +Y (Fwd)   │      ↑      │ +Z (Up)
    │  ←   → │ ±X         │   ←     →   │ Chassis ±
    │    ↓    │ -Y (Back)  │      ↓      │ -Z (Down)
    └─────────┘            └─────────────┘

[OPTIONS] Toggle Manual ⟷ Auto Mode
```

### **Operating Modes**

#### **🤖 Autonomous Mode** (Default)
- System detects weeds using YOLO computer vision
- Plans optimal path to all detected targets
- Moves to hover position above each weed
- Descends to spray height and activates pump
- Retracts and continues to next target

#### **🕹️ Manual Mode** (Press OPTIONS)
- **Hold L1** + move sticks for direct control
- **Left stick**: End effector X/Y movement
- **Right stick**: End effector Z / chassis horizontal
- **R1**: Manual spray activation
- Real-time position feedback and safety limits

## 🔧 Hardware Setup

### **1. ESP32 Firmware**
```bash
# Flash firmware to ESP32
./deploy.sh firmware flash /dev/ttyUSB0

# Monitor serial output
./deploy.sh firmware monitor /dev/ttyUSB0

# Get chip information
./deploy.sh firmware info /dev/ttyUSB0
```

### **2. Delta Robot Calibration**
```bash
# Launch calibration interface
ros2 launch delta_robot_description display.launch.py

# Manual joint control
ros2 launch delta_robot_bringup simulation.launch.py use_joint_gui:=true

# Test basic movement
ros2 topic pub /delta_joint_target std_msgs/Float32MultiArray "data: [0.1, 0.1, 0.1]"
```

### **3. Camera Setup**
```bash
# Test camera feed
ros2 topic echo /camera/image_raw --once

# View camera in GUI
ros2 run rqt_image_view rqt_image_view

# Test detection pipeline
ros2 topic echo /detections --once
```

### **4. PS4 Controller**
```bash
# List available controllers
ros2 run joy joy_enumerate_devices

# Test controller input
ros2 topic echo /joy

# Configure different controller
ros2 launch delta_robot_bringup system.launch.py joy_device_id:=1
```

## 🚀 Development Workflow

### **Simulation Development**
```bash
# Start simulation environment
./deploy.sh sim

# Development with live code reload
./deploy.sh dev

# Run automated tests
./deploy.sh test

# Access development container
docker-compose --profile dev exec delta-robot-dev bash
```

### **Hardware Testing**
```bash
# Flash latest firmware
./deploy.sh firmware flash

# Start hardware mode
./deploy.sh hardware

# Monitor system health
./deploy.sh status

# View logs
./deploy.sh logs
```

### **Custom Modifications**
```bash
# Edit source code
nano ros2_ws/src/delta_robot_control/delta_robot_control/delta_ik.py

# Rebuild and test
cd ros2_ws && colcon build --packages-select delta_robot_control
source install/setup.bash

# Test in simulation
ros2 launch delta_robot_bringup simulation.launch.py
```

## 🔧 Configuration

### **Robot Parameters** (`delta_ik.py`)
```python
# Robot geometry (cm)
f_base_cm: 120.0      # Base triangle side length
e_eff_cm: 40.0        # End effector triangle side
rf_upper_cm: 80.0     # Upper arm length
re_rod_cm: 200.0      # Lower rod length

# Motion parameters
xy_speed_cm_s: 6.0    # XY movement speed
z_speed_cm_s: 6.0     # Z movement speed
spray_time_s: 0.5     # Spray duration per target
```

### **Detection Parameters** (`detector_node.py`)
```python
conf: 0.5             # Detection confidence threshold
iou: 0.5              # IoU threshold for NMS
pixels_per_cm: 5.0    # Camera calibration
weed_class_id: 1      # Target class ID
```

### **Environment Variables**
```bash
# ROS configuration
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Hardware devices
export DEVICE_ESP32=/dev/ttyUSB0
export DEVICE_JOY=/dev/input/js0

# Container settings
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
```

## 📊 Monitoring and Debugging

### **System Status**
```bash
# Overall system status
./deploy.sh status

# ROS topic monitoring
ros2 topic list
ros2 topic hz /delta_joint_position
ros2 topic echo /detections --once

# Node information
ros2 node list
ros2 node info /delta_robot_controller
```

### **Performance Metrics**
```bash
# Control loop timing
ros2 topic hz /delta_joint_target

# Vision pipeline performance
ros2 topic hz /camera/image_raw
ros2 topic hz /detections

# System resource usage
docker stats delta-robot-ros
```

### **Troubleshooting**
```bash
# Check logs
./deploy.sh logs delta-robot-ros

# ESP32 diagnostics
./deploy.sh firmware monitor

# Camera diagnostics
ros2 run rqt_image_view rqt_image_view

# Network diagnostics
ros2 topic list
ros2 service list
```

## 🧪 Testing

### **Automated Tests**
```bash
# Full system test suite
./deploy.sh test

# Simulation-specific tests
./test_simulation.py

# Hardware integration tests (requires ESP32)
ros2 launch delta_robot_bringup system.launch.py simulation_mode:=false
```

### **Manual Test Procedures**

#### **Simulation Testing**
1. Start simulation: `./deploy.sh sim`
2. Verify RViz visualization shows robot model
3. Test PS4 controller input
4. Verify joint movement in simulation
5. Test pump and chassis commands

#### **Hardware Testing**
1. Flash firmware: `./deploy.sh firmware flash`
2. Start hardware mode: `./deploy.sh hardware`
3. Test emergency stop functionality
4. Calibrate joint zero positions
5. Test full autonomous cycle with target detection

## 📚 API Reference

### **Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/delta_joint_target` | `Float32MultiArray` | Joint angle commands (radians) |
| `/delta_joint_position` | `Float32MultiArray` | Current joint positions (radians) |
| `/pump_cmd` | `Float32` | Pump duty cycle (0.0-1.0) |
| `/chassis_cmd` | `Float32` | Chassis movement (-1.0 to 1.0) |
| `/delta_command` | `String` | Text commands ("enable 1", "soft_zero") |
| `/detections` | `Detection2DArray` | Detected weed targets |
| `/camera/image_raw` | `Image` | Raw camera feed |
| `/joint_states` | `JointState` | Joint states for visualization |

### **Services**

| Service | Type | Description |
|---------|------|-------------|
| `/end_effector_control` | `SetBool` | Enable/disable spray pump |

### **Parameters**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `simulation_mode` | `false` | Enable simulation instead of hardware |
| `use_feedback` | `true` | Use ArUco marker feedback |
| `joy_device_id` | `0` | PS4 controller device index |
| `serial_dev` | `/dev/ttyUSB0` | ESP32 serial device |

## 🛡️ Safety

### **Emergency Procedures**
- **Physical E-Stop**: Red emergency button on robot base
- **Software E-Stop**: Circle button on PS4 controller  
- **System Shutdown**: `./deploy.sh stop` or Ctrl+C
- **Firmware Reset**: ESP32 reset button or `./deploy.sh firmware erase`

### **Safety Checklist**
- [ ] Workspace clear of personnel and obstacles
- [ ] Emergency stop accessible and tested
- [ ] Joint limits properly configured
- [ ] Spray system disconnected during testing
- [ ] Proper PPE when handling chemicals
- [ ] Network isolation for field deployment

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guide](CONTRIBUTING.md) for details.

### **Development Setup**
```bash
# Fork the repository
git clone https://github.com/r3tr056/autonomous-delta.git
cd autonomous-delta

# Create development branch
git checkout -b feature/your-feature-name

# Set up development environment
./deploy.sh dev

# Make changes and test
./deploy.sh test

# Submit pull request
```

### **Code Style**
- **Python**: Follow PEP 8, use `black` formatter
- **C++**: Follow ROS 2 style guide
- **Documentation**: Comprehensive docstrings and comments
- **Testing**: Include tests for new features

## 🙏 Acknowledgments

- **ROS 2 Community** for the excellent robotics framework
- **micro-ROS** team for ESP32 integration capabilities
- **Ultralytics** for the YOLO implementation
- **ESP-IDF** team for comprehensive ESP32 development tools
- **Delta Robot Research** community for kinematics references

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/r3tr056/autonomous-delta/issues)
- **Discussions**: [GitHub Discussions](https://github.com/r3tr056autonomous-delta/discussions)  
- **Email**: dangerankur56@gmail.com

---

**🚜🤖 Happy Autonomous Farming! 🌱**