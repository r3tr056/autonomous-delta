#!/bin/bash

# Delta Robot System - One-shot Setup, Build, and (optionally) Run
# Installs system and Python dependencies, prepares micro-ROS agent, builds the workspace,
# and can launch simulation or hardware bringup. Safe to re-run (idempotent where possible).

set -euo pipefail

echo "🤖 Delta Robot System - Full Setup"
echo "=================================="

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="${SCRIPT_DIR}/ros2_ws"

# Defaults
RUN_MODE="none"   # one of: none|sim|hw
INSTALL_DOCKER="1"
INSTALL_MICROROS="1"

usage() {
    cat <<USAGE
Usage: $0 [--run sim|hw] [--no-docker] [--no-micro-ros]

Options:
    --run sim         Build and launch simulation with teleop
    --run hw          Build and launch hardware system (requires camera + MCU connected)
    --no-docker       Skip Docker installation
    --no-micro-ros    Skip micro-ROS agent installation/build
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run)
            RUN_MODE="${2:-none}"; shift 2;;
        --no-docker)
            INSTALL_DOCKER="0"; shift;;
        --no-micro-ros)
            INSTALL_MICROROS="0"; shift;;
        -h|--help)
            usage; exit 0;;
        *)
            echo -e "${YELLOW}Unknown option: $1${NC}"; usage; exit 1;;
    esac
done

# Root check
if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
    echo -e "${RED}❌ This script should not be run as root${NC}"
    exit 1
fi

# Detect ROS distro
if [ -z "${ROS_DISTRO:-}" ]; then
    echo -e "${YELLOW}⚠️  ROS_DISTRO not set. Attempting to detect...${NC}"
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        export ROS_DISTRO=jazzy
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        export ROS_DISTRO=humble
    else
        echo -e "${RED}❌ No ROS installation detected. Please install ROS Humble/Jazzy first.${NC}"
        exit 1
    fi
fi
echo -e "${BLUE}ℹ️  Using ROS ${ROS_DISTRO}${NC}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Detect Raspberry Pi (for Picamera2/libcamera)
IS_RPI=0
if grep -qi 'raspberry pi' /proc/device-tree/model 2>/dev/null; then
    IS_RPI=1
fi

echo -e "${YELLOW}📦 Updating system packages...${NC}"
sudo apt update

# rosdep
if ! command -v rosdep &>/dev/null; then
    echo -e "${YELLOW}📦 Installing rosdep...${NC}"
    sudo apt install -y python3-rosdep
fi
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo -e "${YELLOW}🔧 Initializing rosdep...${NC}"
    sudo rosdep init || true
fi
echo -e "${YELLOW}🔄 Updating rosdep database...${NC}"
rosdep update

echo -e "${YELLOW}📦 Installing base development packages...${NC}"
sudo apt install -y \
    build-essential cmake git curl wget unzip pkg-config \
    python3-dev python3-pip python3-venv python3-setuptools python3-wheel \
    python3-numpy python3-scipy python3-psutil \
    libgl1 libopencv-dev python3-opencv \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libomp-dev libopenblas-dev \
    libvulkan1 vulkan-tools

echo -e "${YELLOW}📦 Installing ROS runtime packages...${NC}"
sudo apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins || true

# NCNN C++ headers (optional) – Python wheel will be installed via pip below
sudo apt install -y libncnn-dev || true

# Install package dependencies via rosdep (workspace)
echo -e "${YELLOW}📦 Resolving ROS package dependencies via rosdep...${NC}"
pushd "${WS_DIR}" >/dev/null
rosdep install --from-paths src --ignore-src -r -y || true
popd >/dev/null

# micro-ROS Agent
if [[ "${INSTALL_MICROROS}" == "1" ]]; then
    echo -e "${YELLOW}📦 Installing micro-ROS agent...${NC}"
    if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-micro-ros-agent"; then
        sudo apt install -y ros-${ROS_DISTRO}-micro-ros-agent || {
            echo -e "${YELLOW}⚠️  micro-ros-agent not available in apt, building from source...${NC}"
            if [ ! -d "/tmp/microros_ws" ]; then
                mkdir -p /tmp/microros_ws/src
                pushd /tmp/microros_ws >/dev/null
                git clone -b "$ROS_DISTRO" https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
                source "/opt/ros/${ROS_DISTRO}/setup.bash"
                rosdep update && rosdep install --from-paths src --ignore-src -y || true
                colcon build
                source install/local_setup.bash
                ros2 run micro_ros_setup create_agent_ws.sh
                ros2 run micro_ros_setup build_agent.sh
                sudo install -m 0755 install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent /usr/local/bin/micro_ros_agent
                popd >/dev/null
            fi
        }
    fi
fi

echo -e "${YELLOW}🐍 Installing Python packages (user scope)...${NC}"
python3 -m pip install --user --upgrade pip
python3 -m pip install --user \
    ultralytics Pillow tqdm pyyaml matplotlib pandas

# NCNN Python wheel (fixes ModuleNotFoundError: ncnn)
python3 -m pip install --user ncnn || {
    echo -e "${YELLOW}⚠️  Could not install ncnn wheel via pip. You may need to build from source.${NC}"
}

# Raspberry Pi specific camera stack
if [[ ${IS_RPI} -eq 1 ]]; then
    echo -e "${YELLOW}🍓 Installing Raspberry Pi camera stack...${NC}"
    sudo apt install -y python3-picamera2 libcamera-dev || true
else
    echo -e "${YELLOW}ℹ️  Not a Raspberry Pi: Picamera2/libcamera Python bindings are typically unavailable. Camera node may not run.${NC}"
fi

# Optional: PS4 controller and access
echo -e "${YELLOW}🎮 Setting up PS4 controller support...${NC}"
sudo apt install -y bluetooth bluez bluez-tools joystick jstest-gtk || true
echo -e "${YELLOW}🔧 Adding user to dialout group for serial access...${NC}"
sudo usermod -a -G dialout "$USER" || true

# Udev rules for PS4 controller
if [ ! -f "/etc/udev/rules.d/99-ps4-controller.rules" ]; then
    echo -e "${YELLOW}🎮 Creating udev rules for PS4 controller...${NC}"
    sudo tee /etc/udev/rules.d/99-ps4-controller.rules >/dev/null << 'EOF'
# PS4 Controller USB
SUBSYSTEM=="input", ATTRS{name}=="Sony Computer Entertainment Wireless Controller", ENV{ID_INPUT_JOYSTICK}="1", TAG+="uaccess"
SUBSYSTEM=="input", ATTRS{name}=="Wireless Controller", ENV{ID_INPUT_JOYSTICK}="1", TAG+="uaccess"

# PS4 Controller Bluetooth
KERNEL=="js[0-9]*", ATTRS{name}=="Sony Computer Entertainment Wireless Controller", TAG+="uaccess"
KERNEL=="js[0-9]*", ATTRS{name}=="Wireless Controller", TAG+="uaccess"
EOF
    sudo udevadm control --reload-rules
    sudo udevadm trigger
fi

# Docker (optional)
if [[ "${INSTALL_DOCKER}" == "1" ]]; then
    echo -e "${YELLOW}🐳 Installing Docker (optional)...${NC}"
    if ! command -v docker &>/dev/null; then
        curl -fsSL https://get.docker.com -o get-docker.sh
        sh get-docker.sh
        sudo usermod -aG docker "$USER"
        rm -f get-docker.sh
        echo -e "${GREEN}✅ Docker installed. Please log out/in for group changes to take effect.${NC}"
    else
        echo -e "${GREEN}✅ Docker already installed${NC}"
    fi
    if ! command -v docker-compose &>/dev/null; then
        sudo apt install -y docker-compose-plugin || true
    fi
fi

# Create helper environment script
echo -e "${YELLOW}📝 Creating workspace setup script...${NC}"
cat > "${SCRIPT_DIR}/setup_workspace.sh" << 'EOF'
#!/bin/bash
# Source this to load ROS and the built workspace
GREEN='\033[0;32m'; BLUE='\033[0;34m'; NC='\033[0m'
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then export ROS_DISTRO=jazzy; elif [ -f "/opt/ros/humble/setup.bash" ]; then export ROS_DISTRO=humble; else echo "❌ No ROS install"; return 1; fi
fi
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${SCRIPT_DIR}/ros2_ws/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/ros2_ws/install/setup.bash" && echo -e "${GREEN}✅ Workspace sourced${NC}"
else
    echo -e "${BLUE}ℹ️  Workspace not built yet. Run 'cd ros2_ws && colcon build' first.${NC}"
fi
alias delta_build='cd $SCRIPT_DIR/ros2_ws && colcon build && source install/setup.bash'
alias delta_clean='cd $SCRIPT_DIR/ros2_ws && rm -rf build install log'
alias delta_sim='ros2 launch delta_robot_bringup simulation_teleop.launch.py'
alias delta_hw='ros2 launch delta_robot_bringup system.launch.py simulation_mode:=false'
EOF
chmod +x "${SCRIPT_DIR}/setup_workspace.sh"

# Build the workspace
echo -e "${YELLOW}🛠️  Building ROS workspace...${NC}"
pushd "${WS_DIR}" >/dev/null
colcon build --symlink-install
popd >/dev/null

# Quick import checks to catch ModuleNotFoundError early
echo -e "${YELLOW}🔎 Verifying key Python imports...${NC}"
python3 - <<'PY'
import sys
ok=True
def check(mod):
    global ok
    try:
        __import__(mod)
        print(f"OK: {mod}")
    except Exception as e:
        ok=False; print(f"FAIL: {mod} -> {e}")
for m in ["cv2","numpy","psutil","ncnn"]:
    check(m)
sys.exit(0 if ok else 1)
PY
echo -e "${GREEN}✅ Import checks passed${NC}"

# Optional run
source "${SCRIPT_DIR}/setup_workspace.sh"
case "${RUN_MODE}" in
    sim)
        echo -e "${GREEN}▶️  Launching simulation (teleop)...${NC}"
        ros2 launch delta_robot_bringup simulation_teleop.launch.py || true
        ;;
    hw)
        if [[ ${IS_RPI} -eq 0 ]]; then
            echo -e "${YELLOW}⚠️  Hardware mode on non-Raspberry Pi: camera_node may fail due to missing Picamera2/libcamera bindings.${NC}"
        fi
        echo -e "${GREEN}▶️  Launching hardware system...${NC}"
        ros2 launch delta_robot_bringup system.launch.py simulation_mode:=false || true
        ;;
    none|*)
        echo -e "${BLUE}ℹ️  Build complete. Source the workspace with: ${GREEN}source ${SCRIPT_DIR}/setup_workspace.sh${NC}"
        echo -e "${BLUE}Try simulation: ${GREEN}delta_sim${NC} or hardware: ${GREEN}delta_hw${NC}"
        ;;
esac

echo -e "${GREEN}🎉 Setup finished${NC}"
