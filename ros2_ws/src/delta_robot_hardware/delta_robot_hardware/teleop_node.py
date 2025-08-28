#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
import time

class PS4Teleop(Node):
	def __init__(self):
		super().__init__('ps4_teleop')

		# Axes index params (defaults follow ROS joy game_controller_node)
		self.declare_parameter('axis_left_x', 0)      # LEFTX
		self.declare_parameter('axis_left_y', 1)      # LEFTY
		self.declare_parameter('axis_right_x', 2)     # RIGHTX
		self.declare_parameter('axis_right_y', 3)     # RIGHTY
		self.declare_parameter('axis_l2', 4)          # TRIGGERLEFT
		self.declare_parameter('axis_r2', 5)          # TRIGGERRIGHT

		# Button index params (defaults per game_controller_node)
		self.declare_parameter('btn_l1', 9)           # LEFTSHOULDER (deadman)
		self.declare_parameter('btn_r1', 10)          # RIGHTSHOULDER (spray)
		self.declare_parameter('btn_cross', 0)        # CROSS (soft zero)
		self.declare_parameter('btn_circle', 1)       # CIRCLE (disable)
		self.declare_parameter('btn_triangle', 3)     # TRIANGLE (enable)
		self.declare_parameter('btn_options', 6)      # START (manual toggle)

		# Teleop scaling
		self.declare_parameter('max_xy_cm_s', 8.0)
		self.declare_parameter('max_z_cm_s', 6.0)
		self.declare_parameter('max_chassis_cmd', 1.0)
		self.declare_parameter('deadzone', 0.1)
		
		self.declare_parameter('jog_z_step_cm', 0.5)
		self.declare_parameter('jog_debounce_s', 0.15)

		

		# Read params
		self.ax_lx = int(self.get_parameter('axis_left_x').value)
		self.ax_ly = int(self.get_parameter('axis_left_y').value)
		self.ax_rx = int(self.get_parameter('axis_right_x').value)
		self.ax_ry = int(self.get_parameter('axis_right_y').value)
		self.ax_l2 = int(self.get_parameter('axis_l2').value)
		self.ax_r2 = int(self.get_parameter('axis_r2').value)

		self.bt_l1 = int(self.get_parameter('btn_l1').value)
		self.bt_r1 = int(self.get_parameter('btn_r1').value)
		self.bt_cross = int(self.get_parameter('btn_cross').value)
		self.bt_circle = int(self.get_parameter('btn_circle').value)
		self.bt_triangle = int(self.get_parameter('btn_triangle').value)
		self.bt_options = int(self.get_parameter('btn_options').value)

		self.max_xy = float(self.get_parameter('max_xy_cm_s').value)
		self.max_z = float(self.get_parameter('max_z_cm_s').value)
		self.deadzone = float(self.get_parameter('deadzone').value)
		self.max_chassis = float(self.get_parameter('max_chassis_cmd').value)

		# Publishers
		qos = QoSProfile(depth=10)
		self.pub_manual = self.create_publisher(Bool, 'delta_teleop/manual', qos)
		self.pub_twist  = self.create_publisher(Twist, 'delta_teleop/twist', qos)
		self.pub_spray  = self.create_publisher(Bool, 'delta_teleop/spray', qos)
		self.pub_enable = self.create_publisher(Bool, 'delta_teleop/enable', qos)
		self.pub_zero   = self.create_publisher(Bool, 'delta_teleop/soft_zero', qos)
		self.pub_chas   = self.create_publisher(Float32, 'chassis_cmd', qos)

		# Subscriber
		self.sub = self.create_subscription(Joy, 'joy', self.cb_joy, qos)

		# State
		self.manual = False
		self.last_options = 0
		self.get_logger().info('PS4 teleop ready (hold L1 to move; left stick delta XY, right stick chassis horiz/Z vert).')

	def dz(self, v):
		return 0.0 if abs(v) < self.deadzone else v

	def cb_joy(self, msg: Joy):
		def ax(i): return msg.axes[i] if 0 <= i < len(msg.axes) else 0.0
		def bt(i): return msg.buttons[i] if 0 <= i < len(msg.buttons) else 0

		# Toggle manual mode on Options edge
		opt = bt(self.bt_options)
		if opt and not self.last_options:
			self.manual = not self.manual
			self.pub_manual.publish(Bool(data=self.manual))
		self.last_options = opt

		deadman = bt(self.bt_l1) > 0

		# Read sticks
		lx = self.dz(ax(self.ax_lx))               # delta X
		ly = self.dz(ax(self.ax_ly))               # delta Y (invert below)
		rx = self.dz(ax(self.ax_rx))               # chassis horiz
		ry = self.dz(ax(self.ax_ry))               # Z vert (invert below)

		# Build twist (cm/s) only when manual + deadman
		tw = Twist()
		if self.manual and deadman:
			tw.linear.x = float(self.max_xy * lx)
			tw.linear.y = float(self.max_xy * (-ly))
			tw.linear.z = float(self.max_z * (-ry))
		else:
			tw.linear.x = tw.linear.y = tw.linear.z = 0.0
		self.pub_twist.publish(tw)

		# Chassis command (manual + deadman; map rx to [-1..1], invert)
		chas_cmd = float(0.0)
		if self.manual and deadman:
			chas_cmd = max(-1.0, min(1.0, rx)) * self.max_chassis
		self.pub_chas.publish(Float32(data=chas_cmd))

		# Spray hold on R1
		self.pub_spray.publish(Bool(data=self.manual and (bt(self.bt_r1) > 0)))

		# Enable/Disable drivers (Triangle/Circle)
		if bt(self.bt_triangle) > 0:
			self.pub_enable.publish(Bool(data=True))
		if bt(self.bt_circle) > 0:
			self.pub_enable.publish(Bool(data=False))

		# Soft zero on Cross
		if bt(self.bt_cross) > 0:
			self.pub_zero.publish(Bool(data=True))

def main():
	rclpy.init()
	node = PS4Teleop()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
