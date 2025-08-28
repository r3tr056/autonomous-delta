#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class EEArucoTracker(Node):
    def __init__(self):
        super().__init__('ee_aruco_tracker')

        # Parameters
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('feedback_topic', 'ee_feedback')
        self.declare_parameter('pixels_per_cm', 5.0)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('display', False)

        img_topic = self.get_parameter('image_topic').value
        fb_topic = self.get_parameter('feedback_topic').value
        self.ppc = float(self.get_parameter('pixels_per_cm').value)
        dict_name = self.get_parameter('aruco_dict').value
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.display = bool(self.get_parameter('display').value)

        # cv_bridge and ROS I/O
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(Image, img_topic, self.cb, qos)
        self.pub = self.create_publisher(PointStamped, fb_topic, 10)

        # OpenCV ArUco dictionary + detector params
        self.aruco_dict = self._get_dictionary(dict_name)
        self.params = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)
        else:
            self.detector = None

        self.get_logger().info(f"EEArucoTracker: dict={dict_name} id={self.marker_id} ppc={self.ppc}")

    def _get_dictionary(self, name):
        # Map string to cv2.aruco predefined dictionary
        dmap = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
        }
        key = dmap.get(name, cv2.aruco.DICT_4X4_50)
        return cv2.aruco.getPredefinedDictionary(key)

    def cb(self, msg: Image):
        # Convert Image -> OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f"cv_bridge error: {e}")
            return

        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0

        # Detect ArUco markers
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.params)
        if ids is None or len(ids) == 0:
            return

        # Find configured marker_id
        ids = ids.flatten()
        for i, mid in enumerate(ids):
            if int(mid) == self.marker_id:
                pts = corners[i].reshape(4, 2)
                mx = float(np.mean(pts[:, 0]))
                my = float(np.mean(pts[:, 1]))
                # Pixel offsets -> cm; v positive up (Cartesian)
                u_cm = (mx - cx) / self.ppc
                v_cm = (cy - my) / self.ppc
                out = PointStamped()
                out.header = msg.header
                out.point.x = u_cm
                out.point.y = v_cm
                out.point.z = 0.0
                self.pub.publish(out)

                if self.display:
                    disp = frame.copy()
                    cv2.aruco.drawDetectedMarkers(disp, [corners[i]], np.array([[mid]]))
                    cv2.drawMarker(disp, (int(cx), int(cy)), (255,255,255), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)
                    cv2.circle(disp, (int(mx), int(my)), 4, (0,255,0), -1)
                    cv2.putText(disp, f"u={u_cm:.1f}cm v={v_cm:.1f}cm", (int(mx)+5, int(my)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                    cv2.imshow("EE ArUco", disp)
                    cv2.waitKey(1)
                break

def main():
    rclpy.init()
    node = EEArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
