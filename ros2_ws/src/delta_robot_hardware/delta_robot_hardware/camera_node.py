#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2
import numpy as np
import time


class PiCameraPublisher(Node):
    def __init__(self):
        super().__init__('pi_camera_publisher')

        # Declare/Read parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('topic', 'camera/image_raw')
        self.declare_parameter('stabilize', False)
        self.declare_parameter('stab_alpha', 0.7)
        self.declare_parameter('stabilized_topic', '')

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.framerate = int(self.get_parameter('framerate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        topic = str(self.get_parameter('topic').value)
        self.stabilize = bool(self.get_parameter('stabilize').value)
        self.stab_alpha = float(self.get_parameter('stab_alpha').value)
        self.stab_topic = str(self.get_parameter('stabilized_topic').value)

        # ROS publisher and bridge
        self.publisher_ = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.pub_stab = None
        if self.stabilize and self.stab_topic:
            self.pub_stab = self.create_publisher(Image, self.stab_topic, qos_profile_sensor_data)

        # Initialize Picamera2 (libcamera)
        self.picam2 = Picamera2()

        # Configure for RGB888 so capture_array() returns HxWx3
        video_cfg = self.picam2.create_video_configuration(
            main={'size': (self.width, self.height), 'format': 'RGB888'},
            controls={'FrameRate': self.framerate}
        )
        self.picam2.configure(video_cfg)
        self.picam2.start()

        # stabilization state
        self.prev_gray = None
        self.off_x = 0.0
        self.off_y = 0.0
        self.last_ts = time.time()

        # Timer based on framerate
        period = 1.0 / float(self.framerate)
        self.timer = self.create_timer(period, self.capture_and_publish)
        self.get_logger().info(
            f'PiCameraPublisher started: {self.width}x{self.height} @ {self.framerate} Hz on topic "{topic}"'
        )

    def _stabilize(self, frame_rgb):
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        if self.prev_gray is None:
            self.prev_gray = gray_blur
            return frame_rgb
        (dx, dy), _ = cv2.phaseCorrelate(np.float32(self.prev_gray), np.float32(gray_blur))
        # EMA smoothing
        self.off_x = self.stab_alpha * self.off_x + (1.0 - self.stab_alpha) * dx
        self.off_y = self.stab_alpha * self.off_y + (1.0 - self.stab_alpha) * dy
        self.prev_gray = gray_blur
        M = np.float32([[1, 0, -self.off_x], [0, 1, -self.off_y]])
        stabilized = cv2.warpAffine(frame_rgb, M, (frame_rgb.shape[1], frame_rgb.shape[0]), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        return stabilized

    def capture_and_publish(self):
        try:
            frame = self.picam2.capture_array("main")  # RGB888
            stamp = self.get_clock().now().to_msg()
            # Publish raw
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.publisher_.publish(msg)
            # Optional stabilized
            if self.stabilize:
                stab = self._stabilize(frame)
                out = self.bridge.cv2_to_imgmsg(stab, encoding='rgb8')
                out.header.stamp = stamp
                out.header.frame_id = self.frame_id
                if self.pub_stab is not None:
                    self.pub_stab.publish(out)
                else:
                    # Replace main stream if no secondary topic provided
                    self.publisher_.publish(out)
        except Exception as e:
            self.get_logger().error(f'Capture/Publish error: {e}')

    def destroy_node(self):
        try:
            self.picam2.stop()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PiCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
