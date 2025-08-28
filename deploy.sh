#!/bin/bash

# Delta Robot System - Deployment and Management Script
# This script provides easy deployment and management of the delta robot system

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.yml"
ROS_IMAGE="delta-robot-ros:latest"
FIRMWARE_IMAGE="delta-robot-firmware:latest"

# Default values
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
DEVICE_ESP32=${DEVICE_ESP32:-/dev/ttyUSB0}
DEVICE_JOY=${DEVICE_JOY:-/dev/input/js0}

# Function to print colored output
print_header() {
    echo -e "${BLUE}🤖 $1${NC}"
    echo -e "${BLUE}$(printf '=%.0s' $(seq 1 ${#1}))${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${CYAN}ℹ️  $1${NC}"
}

# Function to check if Docker is installed and running
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed!"
        echo -e "Please install Docker: ${YELLOW}https://docs.docker.com/get-docker/${NC}"
        exit 1
    fi

    if ! docker info &> /dev/null; then
        print_error "Docker is not running!"
        echo -e "Please start Docker service: ${YELLOW}sudo systemctl start docker${NC}"
        exit 1
    fi

    if ! command -v docker-compose &> /dev/null; then
        print_error "Docker Compose is not installed!"
        echo -e "Please install Docker Compose: ${YELLOW}sudo apt install docker-compose-plugin${NC}"
        exit 1
    fi
}

# Function to set up X11 forwarding
setup_x11() {
    print_info "Setting up X11 forwarding for GUI applications..."

    # Create X11 authentication file
    XAUTH=/tmp/.docker.xauth
    if [ ! -f $XAUTH ]; then
        xauth_list=$(xauth nlist $DISPLAY 2>/dev/null | sed -e 's/^..../ffff/')
        if [ -n "$xauth_list" ]; then
            echo "$xauth_list" | xauth -f $XAUTH nmerge -
        else
            touch $XAUTH
        fi
        chmod a+r $XAUTH
    fi

    # Export display for Docker containers
    export DISPLAY=${DISPLAY:-:0}
    export XAUTHORITY=$XAUTH

    print_success "X11 forwarding configured"
}

# Function to check device permissions
check_devices() {
    print_info "Checking device permissions..."

    # Check ESP32 device
    if [ -e "$DEVICE_ESP32" ]; then
        if [ -r "$DEVICE_ESP32" ] && [ -w "$DEVICE_ESP32" ]; then
            print_success "ESP32 device accessible: $DEVICE_ESP32"
        else
            print_warning "ESP32 device exists but not accessible: $DEVICE_ESP32"
            echo -e "Run: ${YELLOW}sudo usermod -a -G dialout $USER${NC} and re-login"
        fi
    else
        print_warning "ESP32 device not found: $DEVICE_ESP32"
        echo "Available devices:"
        ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  No USB/ACM devices found"
    fi

    # Check joystick device
    if [ -e "$DEVICE_JOY" ]; then
        print_success "Joystick device found: $DEVICE_JOY"
    else
        print_warning "Joystick device not found: $DEVICE_JOY"
        echo "Available joysticks:"
        ls /dev/input/js* 2>/dev/null || echo "  No joystick devices found"
        echo -e "To enumerate joysticks: ${YELLOW}ros2 run joy joy_enumerate_devices${NC}"
    fi
}

# Function to build Docker images
build_images() {
    print_header "Building Docker Images"

    cd "$SCRIPT_DIR"

    print_info "Building ROS environment image..."
    docker-compose build delta-robot-ros

    print_info "Building firmware image..."
    docker-compose --profile firmware build delta-robot-firmware

    print_success "All images built successfully"
}

# Function to start simulation
start_simulation() {
    print_header "Starting Delta Robot Simulation"

    setup_x11

    print_info "Starting ROS container..."
    docker-compose up -d delta-robot-ros

    sleep 3

    print_info "Launching simulation..."
    docker-compose exec delta-robot-ros bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source install/setup.bash && ros2 launch delta_robot_bringup simulation_teleop.launch.py" &

    echo ""
    print_success "Simulation started!"
    print_info "Access container: ${YELLOW}docker-compose exec delta-robot-ros bash${NC}"
    print_info "Run tests: ${YELLOW}docker-compose exec delta-robot-ros /opt/delta_robot_ws/test_system.py${NC}"
    print_info "Stop simulation: ${YELLOW}$0 stop${NC}"
}

# Function to start hardware mode
start_hardware() {
    print_header "Starting Delta Robot Hardware Mode"

    check_devices
    setup_x11

    print_info "Starting ROS container with hardware access..."
    docker-compose up -d delta-robot-ros

    sleep 3

    print_info "Launching hardware mode..."
    docker-compose exec delta-robot-ros bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source install/setup.bash && ros2 launch delta_robot_bringup system.launch.py simulation_mode:=false serial_dev:=$DEVICE_ESP32" &

    echo ""
    print_success "Hardware mode started!"
    print_info "ESP32 device: ${YELLOW}$DEVICE_ESP32${NC}"
    print_info "Access container: ${YELLOW}docker-compose exec delta-robot-ros bash${NC}"
    print_info "Monitor ESP32: ${YELLOW}$0 firmware monitor${NC}"
    print_info "Stop hardware mode: ${YELLOW}$0 stop${NC}"
}

# Function to handle firmware operations
firmware_operations() {
    local operation=${1:-help}
    shift || true

    case "$operation" in
        flash)
            print_header "Flashing ESP32 Firmware"
            device=${1:-$DEVICE_ESP32}
            docker-compose --profile firmware run --rm delta-robot-firmware flash "$device"
            ;;
        monitor)
            print_header "Monitoring ESP32 Serial Output"
            device=${1:-$DEVICE_ESP32}
            docker-compose --profile firmware run --rm delta-robot-firmware monitor "$device"
            ;;
        build)
            print_header "Building ESP32 Firmware"
            docker-compose --profile firmware run --rm delta-robot-firmware build
            ;;
        erase)
            print_header "Erasing ESP32 Flash"
            device=${1:-$DEVICE_ESP32}
            docker-compose --profile firmware run --rm delta-robot-firmware erase "$device"
            ;;
        info)
            print_header "ESP32 Chip Information"
            device=${1:-$DEVICE_ESP32}
            docker-compose --profile firmware run --rm delta-robot-firmware info "$device"
            ;;
        *)
            print_header "ESP32 Firmware Operations"
            echo "Available firmware commands:"
            echo -e "  ${YELLOW}flash [device]${NC}    - Flash firmware to ESP32"
            echo -e "  ${YELLOW}monitor [device]${NC}  - Monitor serial output"
            echo -e "  ${YELLOW}build${NC}             - Rebuild firmware"
            echo -e "  ${YELLOW}erase [device]${NC}    - Erase ESP32 flash"
            echo -e "  ${YELLOW}info [device]${NC}     - Show chip information"
            echo ""
            echo "Examples:"
            echo -e "  ${CYAN}$0 firmware flash${NC}"
            echo -e "  ${CYAN}$0 firmware monitor${NC}"
            echo -e "  ${CYAN}$0 firmware flash /dev/ttyACM0${NC}"
            ;;
    esac
}

# Function to run tests
run_tests() {
    print_header "Running Delta Robot System Tests"

    # Check if simulation is running
    if ! docker-compose ps delta-robot-ros | grep -q "Up"; then
        print_info "Starting simulation for testing..."
        start_simulation
        sleep 10
    fi

    print_info "Running system tests..."
    docker-compose exec delta-robot-ros /opt/delta_robot_ws/test_system.py
}

# Function to show status
show_status() {
    print_header "Delta Robot System Status"

    # Docker status
    echo -e "${BLUE}Docker Containers:${NC}"
    docker-compose ps
    echo ""

    # Image status
    echo -e "${BLUE}Docker Images:${NC}"
    docker images | grep -E "(delta-robot|REPOSITORY)" || echo "No delta-robot images found"
    echo ""

    # Device status
    echo -e "${BLUE}Device Status:${NC}"
    if [ -e "$DEVICE_ESP32" ]; then
        echo -e "ESP32: ${GREEN}$DEVICE_ESP32 (found)${NC}"
    else
        echo -e "ESP32: ${RED}$DEVICE_ESP32 (not found)${NC}"
    fi

    if [ -e "$DEVICE_JOY" ]; then
        echo -e "Joystick: ${GREEN}$DEVICE_JOY (found)${NC}"
    else
        echo -e "Joystick: ${RED}$DEVICE_JOY (not found)${NC}"
    fi
    echo ""

    # ROS topic status (if container is running)
    if docker-compose ps delta-robot-ros | grep -q "Up"; then
        echo -e "${BLUE}Active ROS Topics:${NC}"
        docker-compose exec delta-robot-ros bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source install/setup.bash && ros2 topic list" 2>/dev/null || echo "ROS not responding"
    else
        echo -e "${YELLOW}ROS container not running${NC}"
    fi
}

# Function to stop all services
stop_services() {
    print_header "Stopping Delta Robot System"

    print_info "Stopping containers..."
    docker-compose down

    print_success "All services stopped"
}

# Function to clean everything
clean_system() {
    print_header "Cleaning Delta Robot System"

    read -p "$(echo -e ${YELLOW}⚠️  This will remove all containers, volumes, and images. Continue? [y/N]: ${NC})" -n 1 -r
    echo

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Stopping and removing containers..."
        docker-compose down --volumes --remove-orphans

        print_info "Removing images..."
        docker rmi $ROS_IMAGE $FIRMWARE_IMAGE 2>/dev/null || true

        print_info "Cleaning Docker system..."
        docker system prune -f

        print_success "System cleaned successfully"
    else
        print_info "Clean operation cancelled"
    fi
}

# Function to enter development mode
dev_mode() {
    print_header "Starting Development Mode"

    setup_x11

    print_info "Starting development container..."
    docker-compose --profile dev up -d delta-robot-dev

    print_success "Development container started"
    print_info "Access container: ${YELLOW}docker-compose --profile dev exec delta-robot-dev bash${NC}"
    print_info "Source code mounted from: ${YELLOW}./ros2_ws/src${NC}"
}

# Function to show logs
show_logs() {
    local service=${1:-delta-robot-ros}
    print_header "Showing Logs for $service"
    docker-compose logs -f "$service"
}

# Function to show help
show_help() {
    echo -e "${BLUE}🤖 Delta Robot System Deployment Script${NC}"
    echo -e "${BLUE}==========================================${NC}"
    echo ""
    echo "Usage: $0 <command> [options]"
    echo ""
    echo -e "${YELLOW}Main Commands:${NC}"
    echo -e "  ${GREEN}build${NC}                    - Build all Docker images"
    echo -e "  ${GREEN}sim${NC}                      - Start simulation mode"
    echo -e "  ${GREEN}hardware${NC}                 - Start hardware mode"
    echo -e "  ${GREEN}test${NC}                     - Run system tests"
    echo -e "  ${GREEN}status${NC}                   - Show system status"
    echo -e "  ${GREEN}stop${NC}                     - Stop all services"
    echo -e "  ${GREEN}clean${NC}                    - Clean system (remove containers/images)"
    echo ""
    echo -e "${YELLOW}Firmware Commands:${NC}"
    echo -e "  ${GREEN}firmware flash [device]${NC}  - Flash ESP32 firmware"
    echo -e "  ${GREEN}firmware monitor [device]${NC} - Monitor ESP32 serial"
    echo -e "  ${GREEN}firmware build${NC}           - Build firmware"
    echo -e "  ${GREEN}firmware erase [device]${NC}  - Erase ESP32 flash"
    echo -e "  ${GREEN}firmware info [device]${NC}   - Show ESP32 info"
    echo ""
    echo -e "${YELLOW}Development Commands:${NC}"
    echo -e "  ${GREEN}dev${NC}                      - Start development mode"
    echo -e "  ${GREEN}logs [service]${NC}           - Show service logs"
    echo -e "  ${GREEN}shell${NC}                    - Enter ROS container shell"
    echo ""
    echo -e "${YELLOW}Environment Variables:${NC}"
    echo -e "  ${CYAN}ROS_DOMAIN_ID${NC}            - ROS domain ID (default: 0)"
    echo -e "  ${CYAN}DEVICE_ESP32${NC}             - ESP32 device path (default: /dev/ttyUSB0)"
    echo -e "  ${CYAN}DEVICE_JOY${NC}               - Joystick device path (default: /dev/input/js0)"
    echo ""
    echo -e "${YELLOW}Examples:${NC}"
    echo -e "  ${CYAN}$0 build${NC}                         - Build system"
    echo -e "  ${CYAN}$0 sim${NC}                           - Start simulation"
    echo -e "  ${CYAN}DEVICE_ESP32=/dev/ttyACM0 $0 hardware${NC} - Hardware mode with different device"
    echo -e "  ${CYAN}$0 firmware flash /dev/ttyUSB1${NC}   - Flash firmware to specific device"
    echo -e "  ${CYAN}$0 test${NC}                          - Run automated tests"
    echo ""
}

# Main script logic
main() {
    local command=${1:-help}

    # Check Docker availability
    check_docker

    case "$command" in
        build)
            build_images
            ;;
        sim|simulation)
            start_simulation
            ;;
        hw|hardware)
            start_hardware
            ;;
        firmware)
            shift
            firmware_operations "$@"
            ;;
        test|tests)
            run_tests
            ;;
        status)
            show_status
            ;;
        stop)
            stop_services
            ;;
        clean)
            clean_system
            ;;
        dev|develop)
            dev_mode
            ;;
        logs)
            shift
            show_logs "$@"
            ;;
        shell)
            docker-compose exec delta-robot-ros bash
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Trap for cleanup on exit
cleanup() {
    echo ""
    print_info "Script interrupted"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Run main function
main "$@"
