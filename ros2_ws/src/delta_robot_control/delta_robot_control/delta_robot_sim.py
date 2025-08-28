#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Float32, String
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import numpy as np
import threading
import time
import math


class DeltaRobotSim(Node):
    """
    Delta Robot Simulation Node

    Mimics the ESP32 firmware interface to allow testing the delta robot controller
    in simulation before deploying to real hardware.
    """

    def __init__(self):
        super().__init__('delta_robot_sim')

        # Parameters
        self.declare_parameter('joint_names', [
            'base_brazo1', 'base_brazo2', 'base_brazo3',
            'codo1_a', 'codo1_b', 'codo2_a', 'codo2_b', 'codo3_a', 'codo3_b',
            'forearm1_ee_x', 'forearm1_ee_y'
        ])
        self.declare_parameter('max_joint_velocity', 2.0)  # rad/s
        self.declare_parameter('max_joint_acceleration', 4.0)  # rad/s^2
        self.declare_parameter('position_tolerance', 0.01)  # rad
        self.declare_parameter('control_frequency', 100.0)  # Hz
        self.declare_parameter('enable_constraints', False)  # Simplified - no constraints for now

        self.joint_names = self.get_parameter('joint_names').value
        self.max_vel = float(self.get_parameter('max_joint_velocity').value)
        self.max_accel = float(self.get_parameter('max_joint_acceleration').value)
        self.pos_tol = float(self.get_parameter('position_tolerance').value)
        self.control_freq = float(self.get_parameter('control_frequency').value)
        self.enable_constraints = bool(self.get_parameter('enable_constraints').value)

        self.dt = 1.0 / self.control_freq

        # QoS profiles
        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = QoSReliabilityPolicy.RELIABLE

        qos_best_effort = QoSProfile(depth=10)
        qos_best_effort.history = QoSHistoryPolicy.KEEP_LAST
        qos_best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Publishers (same topics as ESP32 firmware)
        self.pub_joint_pos = self.create_publisher(
            Float32MultiArray, 'delta_joint_position', qos_best_effort)
        self.pub_joint_states = self.create_publisher(
            JointState, 'joint_states', qos_reliable)

        # Subscribers (same topics as ESP32 firmware)
        self.sub_joint_target = self.create_subscription(
            Float32MultiArray, 'delta_joint_target', self.cb_joint_target, qos_reliable)
        self.sub_pump = self.create_subscription(
            Float32, 'pump_cmd', self.cb_pump, qos_reliable)
        self.sub_chassis = self.create_subscription(
            Float32, 'chassis_cmd', self.cb_chassis, qos_reliable)
        self.sub_command = self.create_subscription(
            String, 'delta_command', self.cb_command, qos_reliable)

        # Service (same as ESP32 firmware)
        self.srv_ee = self.create_service(SetBool, 'end_effector_control', self.cb_ee_service)

        # Robot state
        self.actuated_joints = 3  # Only first 3 joints are actuated
        self.num_joints = len(self.joint_names)
        self.current_pos = np.zeros(self.num_joints)  # Current joint positions (rad)
        self.current_vel = np.zeros(self.num_joints)  # Current joint velocities (rad/s)
        self.target_pos = np.zeros(self.actuated_joints)   # Target joint positions (rad) - only actuated
        self.enabled = False
        self.pump_duty = 0.0
        self.chassis_cmd = 0.0

        # Thread synchronization
        self.lock = threading.Lock()

        # Control loop timer
        self.timer = self.create_timer(self.dt, self.control_step)

        self.get_logger().info(f'Delta Robot Simulation started with {self.num_joints} joints at {self.control_freq}Hz')
        self.get_logger().info(f'Joint names: {self.joint_names}')

    def cb_joint_target(self, msg: Float32MultiArray):
        """Handle joint target commands from controller"""
        if len(msg.data) >= self.actuated_joints:
            with self.lock:
                self.target_pos = np.array(msg.data[:self.actuated_joints])
                # Clamp targets to reasonable limits (±60 degrees)
                self.target_pos = np.clip(self.target_pos, -math.pi/3, math.pi/3)

    def cb_pump(self, msg: Float32):
        """Handle pump control commands"""
        with self.lock:
            self.pump_duty = float(msg.data)
            if self.pump_duty > 0:
                self.get_logger().info(f'Pump ON: duty={self.pump_duty:.2f}', throttle_duration_sec=1.0)
            else:
                self.get_logger().info('Pump OFF', throttle_duration_sec=1.0)

    def cb_chassis(self, msg: Float32):
        """Handle chassis movement commands"""
        with self.lock:
            self.chassis_cmd = float(msg.data)
            if abs(self.chassis_cmd) > 0.01:
                self.get_logger().info(f'Chassis: {self.chassis_cmd:.2f}', throttle_duration_sec=1.0)

    def cb_command(self, msg: String):
        """Handle firmware-style string commands"""
        cmd = msg.data.strip().lower()
        if cmd == 'enable 1':
            with self.lock:
                self.enabled = True
            self.get_logger().info('Motors ENABLED')
        elif cmd == 'enable 0':
            with self.lock:
                self.enabled = False
            self.get_logger().info('Motors DISABLED')
        elif cmd == 'soft_zero':
            with self.lock:
                self.current_pos = np.zeros(self.num_joints)
                self.current_vel = np.zeros(self.num_joints)
                self.target_pos = np.zeros(self.actuated_joints)
            self.get_logger().info('Soft zero performed - joints reset to 0')
        else:
            self.get_logger().warn(f'Unknown command: {msg.data}')

    def cb_ee_service(self, request, response):
        """Handle end effector control service calls"""
        with self.lock:
            if request.data:
                self.pump_duty = 0.8  # Default spray duty
                self.get_logger().info('End effector ON (via service)')
            else:
                self.pump_duty = 0.0
                self.get_logger().info('End effector OFF (via service)')
        response.success = True
        response.message = f"End effector {'ON' if request.data else 'OFF'}"
        return response

    def _calculate_passive_joints(self, actuated_positions):
        """Set passive joint positions - simplified for now"""
        # For now, just set all passive joints to zero
        # In a real delta robot implementation, these would be calculated
        # from the parallel mechanism constraints
        passive_positions = np.zeros(self.num_joints - self.actuated_joints)
        return passive_positions

    def control_step(self):
        """Main control loop - simulates joint movement"""
        with self.lock:
            if not self.enabled:
                # When disabled, hold current position
                return

            # Calculate position errors for actuated joints only
            actuated_pos = self.current_pos[:self.actuated_joints]
            pos_error = self.target_pos - actuated_pos

            # Simple trapezoidal velocity profile for actuated joints
            for i in range(self.actuated_joints):
                error = pos_error[i]

                if abs(error) < self.pos_tol:
                    # Close enough to target
                    self.current_vel[i] = 0.0
                    continue

                # Desired velocity towards target
                desired_vel = np.sign(error) * min(self.max_vel, abs(error) / self.dt)

                # Acceleration limit
                vel_error = desired_vel - self.current_vel[i]
                max_vel_change = self.max_accel * self.dt
                vel_change = np.clip(vel_error, -max_vel_change, max_vel_change)

                # Update velocity and position
                self.current_vel[i] += vel_change
                self.current_pos[i] += self.current_vel[i] * self.dt

            # Set passive joint positions to zero (simplified)
            # This allows the transforms to be published correctly
            passive_positions = self._calculate_passive_joints(self.current_pos[:self.actuated_joints])
            self.current_pos[self.actuated_joints:] = passive_positions
            # Set passive joint velocities to zero
            self.current_vel[self.actuated_joints:] = 0.0

            # Publish current joint states (for controller feedback - only actuated joints)
            pos_msg = Float32MultiArray()
            pos_msg.data = [float(pos) for pos in self.current_pos[:self.actuated_joints]]
            self.pub_joint_pos.publish(pos_msg)

            # Publish joint states (for RViz visualization - all joints)
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = self.joint_names
            js_msg.position = [float(pos) for pos in self.current_pos]
            js_msg.velocity = [float(vel) for vel in self.current_vel]
            js_msg.effort = [0.0] * self.num_joints  # No effort simulation

            self.pub_joint_states.publish(js_msg)

    def get_robot_state_info(self):
        """Get current robot state for logging/debugging"""
        with self.lock:
            return {
                'enabled': self.enabled,
                'positions': self.current_pos.copy(),
                'targets': self.target_pos.copy(),
                'velocities': self.current_vel.copy(),
                'pump_duty': self.pump_duty,
                'chassis_cmd': self.chassis_cmd
            }


def main(args=None):
    rclpy.init(args=args)

    node = DeltaRobotSim()

    # Add signal handler for graceful shutdown
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Simulation stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
