#!/usr/bin/env python3

import math
from socket import timeout
import time
import threading
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, Twist
from std_srvs.srv import SetBool


class DeltaIK:
	"""Rotary delta IK (Clavel) closed-form"""

	def __init__(self, f, e, rf, re, z_down_negative=True, shoulder_deg_offset=(0.0, 0.0, 0.0), shoulder_sign=(1.0, 1.0, 1.0), out_radians=True):
		self.f = float(f)
		self.e = float(e)
		self.rf = float(rf)
		self.re = float(re)

		self.z_down_negative = bool(z_down_negative)
		self.off = tuple(float(a) for a in shoulder_deg_offset)
		self.sign = tuple(float(s) for s in shoulder_sign)
		self.out_radians = bool(out_radians)

		self.sqrt3 = math.sqrt(3)
		self.pi = math.pi
		self.sin120 = self.sqrt3 / 2.0
		self.cos120 = -0.5
		self.tan30 = 1.0 / self.sqrt3

		self.t = (self.f - self.e) * self.tan30 / 2.0


	def _ik_1arm(self, x0, y0, z0):
		if abs(z0) < 1e-6:
			return None
		y1 = -self.t
		y0p = y0 - self.t
		a = (x0*x0 + y0p*y0p + z0*z0 + self.rf*self.rf - self.re*self.re - y1*y1) / (2.0 * z0)
		b = (y1 - y0p) / z0
		d = -(a + b*y1)**2 + self.rf * (b*b*self.rf + self.rf)
		if d < 0:
			return None
		yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1.0)
		zj = a + b*yj
		theta = math.degrees(math.atan2(-zj, (y1 - yj)))
		return theta

	def inverse(self, x, y, z):
		z0 = z if self.z_down_negative else -z

		# Arm 1: No rotations
		t1 = self._ik_1arm(x, y, z0)
		if t1 is None:
			return None

		# Arm 2: Rotate target by -120 deg
		x2 = x * self.cos120 + y * self.sin120
		y2 = -x * self.sin120 + y * self.cos120
		t2 = self._ik_1arm(x2, y2, z0)
		if t2 is None:
			return None

		# Arm 3 : rotate target by +120 deg
		x3 = x * self.cos120 - y * self.sin120
		y3 = x * self.sin120 + y * self.cos120
		t3 = self._ik_1arm(x3, y3, z0)
		if t3 is None:
			return None

		# Apply shoulder sign and offsets, convert to radians
		td0 = (self.sign[0] * t1 + self.off[0]) * math.pi / 180.0
		td1 = (self.sign[1] * t2 + self.off[1]) * math.pi / 180.0
		td2 = (self.sign[2] * t3 + self.off[2]) * math.pi / 180.0
		return (td0, td1, td2)


class DeltaRobotController(Node):

	def __init__(self):
		super().__init__('delta_robot_controller')

		# Geometry (cm)
		self.declare_parameter('f_base_cm', 120.0)
		self.declare_parameter('e_eff_cm', 40.0)
		self.declare_parameter('rf_upper_cm', 80.0)
		self.declare_parameter('re_rod_cm', 200.0)

		# Motion & timing
		self.declare_parameter('hover_z_cm', -8.0)
		self.declare_parameter('spray_z_cm', -12.0)
		self.declare_parameter('xy_speed_cm_s', 6.0)
		self.declare_parameter('z_speed_cm_s', 6.0)
		self.declare_parameter('control_rate_hz', 100.0)
		self.declare_parameter('zone_radius_cm', 1.5)
		self.declare_parameter('align_gain', 0.3)
		self.declare_parameter('align_timeout_s', 2.0)

		# Topics & selection
		self.declare_parameter('detections_topic', 'detections')
		self.declare_parameter('joint_target_topic', 'delta_joint_target')
		self.declare_parameter('joint_state_topic', 'delta_joint_position')
		self.declare_parameter('ee_feedback_topic', 'ee_feedback')
		self.declare_parameter('use_feedback', True)
		self.declare_parameter('weed_class_id', 1)
		self.declare_parameter('min_confidence', 0.5)

		# Sprayer interface
		self.declare_parameter('pump_topic', 'pump_cmd')
		self.declare_parameter('use_pump_topic', True)
		self.declare_parameter('spray_duty', 0.8)
		self.declare_parameter('spray_time_s', 0.5)
		self.declare_parameter('ee_service', 'end_effector_control')

		# path replanning
		self.declare_parameter('replan_new_targets', 3)
		self.declare_parameter('target_stale_s', 5.0)

		self.declare_parameter('chassis_auto_speed', 0.2)
		self.chassis_auto_speed = float(self.get_parameter('chassis_auto_speed').value)

		# params
		f = float(self.get_parameter('f_base_cm').value)
		e = float(self.get_parameter('e_eff_cm').value)
		rf = float(self.get_parameter('rf_upper_cm').value)
		re = float(self.get_parameter('re_rod_cm').value)
		self.z_hover = float(self.get_parameter('hover_z_cm').value)
		self.z_spray = float(self.get_parameter('spray_z_cm').value)
		self.xy_speed = float(self.get_parameter('xy_speed_cm_s').value)
		self.z_speed = float(self.get_parameter('z_speed_cm_s').value)
		self.rate_hz = float(self.get_parameter('control_rate_hz').value)
		self.zone_r = float(self.get_parameter('zone_radius_cm').value)
		self.align_gain = float(self.get_parameter('align_gain').value)
		self.align_timeout_s = float(self.get_parameter('align_timeout_s').value)
		self.topic_det = self.get_parameter('detections_topic').value
		self.topic_target = self.get_parameter('joint_target_topic').value
		self.topic_state = self.get_parameter('joint_state_topic').value
		self.topic_fb = self.get_parameter('ee_feedback_topic').value
		self.use_feedback = bool(self.get_parameter('use_feedback').value)
		self.weed_cid = int(self.get_parameter('weed_class_id').value)
		self.min_conf = float(self.get_parameter('min_confidence').value)
		self.topic_pump = self.get_parameter('pump_topic').value
		self.use_pump_topic = bool(self.get_parameter('use_pump_topic').value)
		self.spray_duty = float(self.get_parameter('spray_duty').value)
		self.spray_time_s = float(self.get_parameter('spray_time_s').value)
		self.ee_service = self.get_parameter('ee_service').value
		self.replan_thresh = int(self.get_parameter('replan_new_targets').value)
		self.target_stale_s = float(self.get_parameter('target_stale_s').value)

		self.ik = DeltaIK(f=f, e=e, rf=rf, re=re)

		# Publishers
		self.pub_joint = self.create_publisher(Float32MultiArray, self.topic_target, 10)
		self.pub_pump = self.create_publisher(Float32, self.topic_pump, 10) if self.use_pump_topic else None
		self.cli_ee = self.create_client(SetBool, self.ee_service)
		self.pub_fw_cmd = self.create_publisher(String, 'delta_command', 10)
		self.pub_chas = self.create_publisher(Float32, 'chassis_cmd', 10)

		# Subscriptions
		qos_best_effort = QoSProfile(depth=10)
		qos_best_effort.history = QoSHistoryPolicy.KEEP_LAST
		qos_best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT

		self.sub_state = self.create_subscription(Float32MultiArray, self.topic_state, self.cb_joint_state, qos_best_effort)
		self.sub_det = self.create_subscription(Detection2DArray, self.topic_det, self.cb_detections, 10)
		self.sub_fb = None
		if self.use_feedback:
			self.sub_fb = self.create_subscription(PointStamped, self.topic_fb, self.cb_fb, 10)

		# Teleop subscribers
		self.sub_jogz = self.create_subscription(Float32, 'delta_teleop/jog_z', self.cb_jogz, 10)
		self.sub_manual = self.create_subscription(Bool, 'delta_teleop/manual', self.cb_manual, 10)
		self.sub_twist  = self.create_subscription(Twist, 'delta_teleop/twist', self.cb_twist, 10)
		self.sub_spray  = self.create_subscription(Bool, 'delta_teleop/spray', self.cb_spray, 10)
		self.sub_enable = self.create_subscription(Bool, 'delta_teleop/enable', self.cb_enable, 10)
		self.sub_zero   = self.create_subscription(Bool, 'delta_teleop/soft_zero', self.cb_soft_zero, 10)

		# State
		self.ctrl_dt = 1.0 / max(1.0, self.rate_hz)
		self.curr_q = [0.0, 0.0, 0.0]
		self.curr_xyz = [0.0, 0.0, self.z_hover]
		self.ee_xy: Optional[Tuple[float, float]] = None
		self.targets: Dict[int, Dict[str, float]] = {}
		self.path: List[int] = []     # list of tracking_ids in planned order
		self.new_since_plan = 0
		self.active_motion = False
		self.stop_motion = False
		self.teleop_manual = False
		self.teleop_twist = False
		self.teleop_twist = Twist()
		self.teleop_spraying = False

		self.lock = threading.Lock()

		# Control thread
		threading.Thread(target=self.run, daemon=True).start()
		self.get_logger().info("DeltaWeedController ready", throttle_duration_sec=2.0)

	def cb_manual(self, msg: Bool):
		self.teleop_manual = bool(msg.data)

	def cb_twist(self, msg: Twist):
		self.teleop_twist = msg

	def cb_spray(self, msg: Bool):
		self.teleop_spraying = bool(msg.data)

	def cb_jogz(self, msg: Float32):
		step = float(msg.data)
		with self.lock:
			x, y, z = self.curr_xyz
		nz = max(self.z_spray - 2.0, min(self.z_hover + 2.0, z + step))
		q = self.ik.inverse(x, y, nz)
		if q is not None:
			self.publish_joints(q)
			with self.lock:
				self.curr_xyz = [x, y, nz]

	def cb_enable(self, msg: Bool):
		cmd = "enable 1" if msg.data else "enable 0"
		self.pub_fw_cmd.publish(String(data=cmd))

	def cb_soft_zero(self, msg: Bool):
		if msg.data:
			q = self.ik.inverse(0.0, 0.0, self.z_hover)
			if q is not None:
				self.publish_joints(q)
				with self.lock:
					self.curr_xyz = [0.0, 0.0, self.z_hover]
			self.pub_fw_cmd.publish(String(data="soft_zero"))

	def cb_joint_state(self, msg: Float32MultiArray):
		if len(msg.data) >= 3:
			with self.lock:
				self.curr_q = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]

	def cb_fb(self, msg: PointStamped):
		with self.lock:
			self.ee_xy = (float(msg.point.x), float(msg.point.y))

	def cb_detections(self, msg: Detection2DArray):
		now = time.time()
		added = 0
		with self.lock:
			for det in msg.detections:
				tid = getattr(det, 'tracking_id', 0)
				if tid is None or tid == 0:
					bx = float(det.bbox.center.position.x)
					by = float(det.bbox.center.position.y)
					tid = (int(bx * 73856093) ^ int(by * 19349663)) & 0x7fffffff

				if len(det.results) == 0:
					continue

				hyp = det.results[0]
				cid = int(getattr(hyp, 'id', -1))
				conf = float(getattr(hyp, 'score', 0.0))
				if cid != self.weed_cid or conf < self.min_conf:
					continue

				# (u, v) in cm from image center
				x = float(hyp.pose.pose.position.x)
				y = float(hyp.pose.pose.position.y)
				if tid not in self.targets:
					added += 1
				self.targets[tid] = {'x': x, 'y': y, 'score': conf, 'last_seen': now}

			# Drop stale targets
			stale = [tid for tid, t in self.targets.items() if (now - t['last_seen']) > self.target_stale_s]
			for tid in stale:
				if tid in self.targets:
					del self.targets[tid]
			self.new_since_plan += added

	def publish_joints(self, q: Tuple[float, float, float]):
		m = Float32MultiArray()
		m.data = [float(q[0]), float(q[1]), float(q[2])]
		self.pub_joint.publish(m)

	def pump_on(self):
		if self.pub_pump is not None:
			self.pub_pump.publish(Float32(data=float(self.spray_duty)))
		else:
			if self.cli_ee.wait_for_service(timeout_sec=0.2):
				req = SetBool.Request()
				req.data = True
				self.cli_ee.call_async(req)

	def pump_off(self):
		if self.pub_pump is not None:
			self.pub_pump.publish(Float32(data=0.0))
		else:
			if self.cli_ee.wait_for_service(timeout_sec=0.2):
				req = SetBool.Request()
				req.data = False
				self.cli_ee.call_async(req)

	def plan_line(self, start: Tuple[float, float, float], goal: Tuple[float, float, float], speed: float) -> List[Tuple[float, float, float]]:
		sx, sy, sz = start
		gx, gy, gz = goal
		dx, dy, dz = (gx - sx), (gy - sy), (gz - sz)
		dist = math.sqrt(dx * dx + dy * dy + dz * dz)
		if dist < 1e-6:
			return []
		steps = max(1, int(math.ceil(dist / max(1e-3, speed * self.ctrl_dt))))
		return [(sx + (i / steps) * dx, sy + (i / steps) * dy, sz + (i / steps) * dz) for i in range(1, steps + 1)]

	def move_cartesian(self, waypoints: List[Tuple[float, float, float]]) -> bool:
		for (x, y, z) in waypoints:
			if self.stop_motion:
				return False
			q = self.ik.inverse(x, y, z)
			if q is None:
				self.get_logger().warning(f"IK failed at {x:.1f},{y:.1f},{z:.1f}; segment abort")
				return False
			self.publish_joints(q)
			with self.lock:
				self.curr_xyz = [x, y, z]
			time.sleep(self.ctrl_dt)
		return True

	def align_in_zone(self, tx: float, ty: float) -> bool:
		if not self.use_feedback:
			with self.lock:
				x, y, _ = self.curr_xyz
			return (abs(tx - x) <= self.zone_r) and (abs(ty - y) <= self.zone_r)

		deadline = time.time() + self.align_timeout_s
		while time.time() < deadline:
			with self.lock:
				m = self.ee_xy
				x, y, z = self.curr_xyz
			if m is None:
				time.sleep(self.ctrl_dt)
				continue

			mx, my = m
			ex = tx - mx
			ey = ty - my
			if abs(ex) <= self.zone_r and abs(ey) <= self.zone_r:
				return True

			nx = x + self.align_gain * ex
			ny = y + self.align_gain * ey
			q = self.ik.inverse(nx, ny, self.z_hover)
			if q is None:
				self.align_gain *= 0.5
				time.sleep(self.ctrl_dt)
				continue
			self.publish_joints(q)
			with self.lock:
				self.curr_xyz = [nx, ny, self.z_hover]
			time.sleep(self.ctrl_dt)
		return False

	def nearest_neighbor_path(self, start_xy: Tuple[float, float], targets: Dict[int, Dict[str, float]]) -> List[int]:
		# Greedy NN ordering (O(n^2)), adequate for small dynamic sets
		unvisited = set(targets.keys())
		order: List[int] = []
		cx, cy = start_xy
		while unvisited:
			best = None
			best_d = 1e9
			for tid in unvisited:
				tx, ty = targets[tid]['x'], targets[tid]['y']
				d = math.hypot(tx - cx, ty - cy)
				if d < best_d:
					best_d = d; best = tid
			order.append(best)
			cx, cy = targets[best]['x'], targets[best]['y']
			unvisited.remove(best)
		return order

	def build_or_rebuild_path(self):
		with self.lock:
			tmap = dict(self.targets)
			self.new_since_plan = 0
			x, y, _ = self.curr_xyz
			start_xy = (x, y)
		if not tmap:
			return []
		return self.nearest_neighbor_path(start_xy, tmap)

	def step_manual(self):
		with self.lock:
			x, y, z = self.curr_xyz
			vx = float(self.teleop_twist.linear.x)
			vy = float(self.teleop_twist.linear.y)
			vz = float(self.teleop_twist.linear.z)

		nx = x + vx * self.ctrl_dt
		ny = y + vy * self.ctrl_dt
		nz = z + vz * self.ctrl_dt

		# clamp withing safe workspace limits
		nx = max(-15.0, min(15.0, nx))
		ny = max(-15.0, min(15.0, ny))
		nz = max(self.z_spray - 2.0, min(self.z_hover + 2.0, nz))
		q = self.ik.inverse(nx, ny, nz)
		if q is None:
			return
		self.publish_joints(q)
		with self.lock:
			self.curr_xyz = [nx, ny, nz]
		# Spray hold
		if self.teleop_spraying:
			self.pump_on()
		else:
			self.pump_off()

	def execute_target(self, tid):
		# Read snapshot
		with self.lock:
			if tid not in self.targets:
				return
			t = dict(self.targets[tid])
			start = tuple(self.curr_xyz)
		tx, ty = t['x'], t['y']
		hover = (tx, ty, self.z_hover)
		if not self.move_cartesian(self.plan_line(start, hover, self.xy_speed)):
			return
		# Align within zone (deadband)
		self.align_in_zone(tx, ty)
		# Touchdown
		touch = (tx, ty, self.z_spray)
		if not self.move_cartesian(self.plan_line(hover, touch, self.z_speed)):
			return
		# Spray
		self.pump_on()
		time.sleep(self.spray_time_s)
		self.pump_off()
		# Retract
		self.move_cartesian(self.plan_line(touch, hover, self.z_speed))

	def run(self):
		while rclpy.ok():
			# Publish slow chassis creep in auto; zero in manual (teleop drives chassis directly)
			if not self.teleop_manual:
				self.pub_chas.publish(Float32(data=self.chassis_auto_speed))
			else:
				self.pub_chas.publish(Float32(data=0.0))

			# Manual mode
			if self.teleop_manual:
				self.step_manual()
				time.sleep(self.ctrl_dt)
				continue

			# Auto mode
			with self.lock:
				have_path = len(self.path) > 0
			if not have_path:
				self.path = self.build_or_rebuild_path()

			while self.path and rclpy.ok() and not self.teleop_manual:
				with self.lock:
					if self.new_since_plan > self.replan_thresh:
						self.stop_motion = True
				if self.stop_motion:
					self.stop_motion = False
					break
				tid = self.path.pop(0)
				self.active_motion = True
				try:
					self.execute_target(tid)
				except Exception as e:
					self.get_logger().error(f"Target {tid} error: {e}")
				finally:
					self.active_motion = False

				time.sleep(0.02)

			with self.lock:
				if not self.path or self.new_since_plan > self.replan_thresh:
					self.path = self.build_or_rebuild_path()

			time.sleep(0.01)



def main():
	rclpy.init()
	node = DeltaRobotController()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
