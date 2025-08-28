#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import cv2
import numpy as np
import ncnn
import time
import itertools
import os
import math

def iou_xyxy(a, b):
    # a,b: [x1,y1,x2,y2]
    xA = max(a[0], b[0]); yA = max(a[1], b[1])
    xB = min(a[2], b[2]); yB = min(a[3], b[3])
    w = max(0.0, xB - xA); h = max(0.0, yB - yA)
    inter = w * h
    if inter <= 0:
        return 0.0
    areaA = max(0.0, (a[2] - a[0])) * max(0.0, (a[3] - a[1]))
    areaB = max(0.0, (b[2] - b[0])) * max(0.0, (b[3] - b[1]))
    union = areaA + areaB - inter
    if union <= 0:
        return 0.0
    return inter / union

class Track:

    _ids = itertools.count(1)

    def __init__(self, box_xyxy, cls_id, conf, ts, ema_alpha=0.6):
        self.id = next(Track._ids)
        self.box = np.array(box_xyxy, dtype=np.float32)
        self.cls = int(cls_id)
        self.conf = float(conf)
        self.hits = 1
        self.time_since_update = 0
        self.last_ts = ts
        # velocity (cx,cy) pixels/sec
        cx = 0.5 * (self.box[0] + self.box[2])
        cy = 0.5 * (self.box[1] + self.box[3])
        self.cx = cx; self.cy = cy
        self.vx = 0.0; self.vy = 0.0
        self.alpha = float(ema_alpha)

    def predict(self, now_ts):
        dt = max(0.0, now_ts - self.last_ts)
        self.cx += self.vx * dt
        self.cy += self.vy * dt
        w = max(1.0, self.box[2] - self.box[0])
        h = max(1.0, self.box[3] - self.box[1])
        nx1 = self.cx - 0.5 * w; ny1 = self.cy - 0.5 * h
        nx2 = self.cx + 0.5 * w; ny2 = self.cy + 0.5 * h
        self.box = np.array([nx1, ny1, nx2, ny2], dtype=np.float32)
        self.time_since_update += 1
        self.last_ts = now_ts

    def update(self, det_box, det_conf, now_ts):
        # Update velocity and EMA-smoothed box
        dt = max(1e-3, now_ts - self.last_ts)
        dcx = 0.5 * (det_box[0] + det_box[2])
        dcy = 0.5 * (det_box[1] + det_box[3])
        vx_new = (dcx - self.cx) / dt
        vy_new = (dcy - self.cy) / dt
        self.vx = self.alpha * self.vx + (1.0 - self.alpha) * vx_new
        self.vy = self.alpha * self.vy + (1.0 - self.alpha) * vy_new
        # EMA on box corners for smoothing
        self.box = self.alpha * self.box + (1.0 - self.alpha) * np.array(det_box, dtype=np.float32)
        self.cx = 0.5 * (self.box[0] + self.box[2])
        self.cy = 0.5 * (self.box[1] + self.box[3])
        self.conf = max(self.conf, float(det_conf))
        self.hits += 1
        self.time_since_update = 0
        self.last_ts = now_ts

class NCNNDetectorNode(Node):
    def __init__(self):
        super().__init__('ncnn_detector_node')

        # Parameters
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('detections_topic', 'detections')  # tracked outputs
        self.declare_parameter('annotated_topic', 'camera/image_annotated')
        self.declare_parameter('model_param_path', 'models/weed_model/ncnn_model/model.ncnn.param')
        self.declare_parameter('model_bin_path', 'models/weed_model/ncnn_model/model.ncnn.bin')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('iou', 0.5)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('pixels_per_cm', 5.0)
        # Tracking config
        self.declare_parameter('track_iou_thresh', 0.3)
        self.declare_parameter('max_age', 10)       # frames to keep unmatched
        self.declare_parameter('min_hits', 3)       # frames before confirmed
        self.declare_parameter('ema_alpha', 0.6)    # smoothing
        self.declare_parameter('weed_class_id', 1)  # only track weed

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value
        self.model_param_path = self.get_parameter('model_param_path').get_parameter_value().string_value
        self.model_bin_path = self.get_parameter('model_bin_path').get_parameter_value().string_value
        self.conf = float(self.get_parameter('conf').get_parameter_value().double_value)
        self.iou = float(self.get_parameter('iou').get_parameter_value().double_value)
        self.imgsz = int(self.get_parameter('imgsz').get_parameter_value().integer_value)
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.pixels_per_cm = float(self.get_parameter('pixels_per_cm').get_parameter_value().double_value)

        self.track_iou_thresh = float(self.get_parameter('track_iou_thresh').get_parameter_value().double_value)
        self.max_age = int(self.get_parameter('max_age').get_parameter_value().integer_value)
        self.min_hits = int(self.get_parameter('min_hits').get_parameter_value().integer_value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').get_parameter_value().double_value)
        self.weed_class_id = int(self.get_parameter('weed_class_id').get_parameter_value().integer_value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self.pub_ann = self.create_publisher(Image, annotated_topic, 10)
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos_profile_sensor_data)

        # Initialize NCNN network
        self.net = ncnn.Net()

        # Set optimal parameters for Raspberry Pi 4
        self.net.opt.use_vulkan_compute = False  # Disable GPU on RPi4
        self.net.opt.num_threads = 4  # Use all 4 cores
        self.net.opt.use_winograd_convolution = True
        self.net.opt.use_sgemm_convolution = True
        self.net.opt.use_int8_inference = False  # Keep FP16 for accuracy
        self.net.opt.use_fp16_packed = True
        self.net.opt.use_fp16_storage = True
        self.net.opt.use_fp16_arithmetic = True
        self.net.opt.use_packing_layout = True
        self.net.opt.use_shader_pack8 = False
        self.net.opt.use_image_storage = False

        # Construct absolute paths
        package_path = os.path.dirname(os.path.dirname(__file__))
        param_path = os.path.join(package_path, self.model_param_path)
        bin_path = os.path.join(package_path, self.model_bin_path)

        # Load model
        if not os.path.exists(param_path):
            self.get_logger().error(f'Model param file not found: {param_path}')
            raise FileNotFoundError(f'Model param file not found: {param_path}')
        if not os.path.exists(bin_path):
            self.get_logger().error(f'Model bin file not found: {bin_path}')
            raise FileNotFoundError(f'Model bin file not found: {bin_path}')

        ret = self.net.load_param(param_path)
        if ret != 0:
            raise RuntimeError(f'Failed to load model params: {ret}')

        ret = self.net.load_model(bin_path)
        if ret != 0:
            raise RuntimeError(f'Failed to load model weights: {ret}')

        self.get_logger().info(f'NCNN model loaded: param={param_path}, bin={bin_path}')
        self.get_logger().info(f'Detection config: conf={self.conf} iou={self.iou} imgsz={self.imgsz} ppc={self.pixels_per_cm}')

        self.tracks = []
        self.prev_gray = None
        self.dx = 0.0; self.dy = 0.0
        self.stab_alpha = 0.7  # for dx,dy EMA
        self.last_ts = time.time()

    def _estimate_translation(self, frame_bgr):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        g = cv2.GaussianBlur(gray, (5, 5), 0)
        if self.prev_gray is None:
            self.prev_gray = g
            return 0.0, 0.0
        (dx, dy), _ = cv2.phaseCorrelate(np.float32(self.prev_gray), np.float32(g))
        # EMA smoothing for global shift
        self.dx = self.stab_alpha * self.dx + (1.0 - self.stab_alpha) * dx
        self.dy = self.stab_alpha * self.dy + (1.0 - self.stab_alpha) * dy
        self.prev_gray = g
        return self.dx, self.dy

    def _associate(self, dets, now_ts):
        # Predict existing tracks
        for t in self.tracks:
            t.predict(now_ts)

        # Build IoU match greedy
        unmatched_tracks = set(range(len(self.tracks)))
        unmatched_dets = set(range(len(dets)))
        matches = []
        # Precompute IoU
        iou_mat = np.zeros((len(self.tracks), len(dets)), dtype=np.float32)
        for i, t in enumerate(self.tracks):
            for j, d in enumerate(dets):
                iou_mat[i, j] = iou_xyxy(t.box, d['box'])
        # Greedy: pick best IoU pairs above threshold
        while True:
            if len(unmatched_tracks) == 0 or len(unmatched_dets) == 0:
                break
            # find global best
            # best = (None, None, self.track_iou_thresh)
            # for i in unmatched_tracks:
            #     j = int(np.argmax(iou_mat[i])) if len(unmatched_dets) > 0 else -1
            #     if j in unmatched_dets and iou_mat[i, j] > best[2]:
            #         best = (i, j, iou_mat[i, j])
            # if best is None:
            #     break
            best_i, best_j, best_score = None, None, self.track_iou_thresh
            for i in list(unmatched_tracks):
                for j in list(unmatched_dets):
                    score = iou_mat[i, j]
                    if score > best_score:
                        best_i, best_j, best_score = i, j, score
            if best_i is None:
                break
            matches.append((best_i, best_j))
            unmatched_tracks.remove(best_i)
            unmatched_dets.remove(best_j)

        # Update matched
        for i, j in matches:
            d = dets[j]
            self.tracks[i].update(d['box'], d['conf'], now_ts)
            self.tracks[i].cls = d['cls']

        # Create new tracks for unmatched dets
        for j in unmatched_dets:
            d = dets[j]
            self.tracks.append(Track(d['box'], d['cls'], d['conf'], now_ts, ema_alpha=self.ema_alpha))

        # Age and remove stale
        new_tracks = []
        for i, t in enumerate(self.tracks):
            if t.time_since_update <= self.max_age:
                new_tracks.append(t)
        self.tracks = new_tracks

    def image_cb(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        h, w = frame_bgr.shape[:2]
        cx_img = w / 2.0; cy_img = h / 2.0
        now_ts = time.time()

        # Global translation estimate to compensate camera creep
        dx, dy = self._estimate_translation(frame_bgr)

        # Run NCNN detector
        try:
            dets = self._run_ncnn_inference(frame_bgr, dx, dy)
        except Exception as e:
            self.get_logger().error(f'NCNN inference error: {e}')
            return

        # Associate/update tracks
        self._associate(dets, now_ts)

        # Prepare outputs
        out = Detection2DArray(); out.header = msg.header
        ann = frame_bgr.copy()
        for t in self.tracks:
            if t.hits < self.min_hits:
                continue
            x1, y1, x2, y2 = t.box
            # Compute (u,v) cm from image center using compensated centroid reprojected to current frame
            bx = 0.5 * (x1 + x2) + dx
            by = 0.5 * (y1 + y2) + dy
            u_cm = (bx - cx_img) / self.pixels_per_cm
            v_cm = (cy_img - by) / self.pixels_per_cm

            det = Detection2D()
            det.header = msg.header
            bbox = BoundingBox2D()
            bbox.center.x = float(bx); bbox.center.y = float(by); bbox.center.theta = 0.0
            bbox.size_x = float(max(0.0, x2 - x1))
            bbox.size_y = float(max(0.0, y2 - y1))
            det.bbox = bbox
            hyp = ObjectHypothesisWithPose()
            hyp.class_id = int(t.cls)
            hyp.score = float(t.conf)
            hyp.pose.pose.position.x = float(u_cm)
            hyp.pose.pose.position.y = float(v_cm)
            hyp.pose.pose.position.z = 0.0
            det.results.append(hyp)
            det.id = int(t.id)
            out.detections.append(det)

            # Annotate
            cv2.rectangle(ann, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(ann, f'id:{t.id} conf:{t.conf:.2f} u:{u_cm:.1f} v:{v_cm:.1f}', (int(x1), max(15, int(y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
            cv2.circle(ann, (int(bx), int(by)), 3, (0, 255, 0), -1)

        # Publish tracked detections and annotated frame
        self.pub.publish(out)
        try:
            ann_msg = self.bridge.cv2_to_imgmsg(ann, encoding='bgr8')
            ann_msg.header = msg.header
            self.pub_ann.publish(ann_msg)
        except Exception as e:
            self.get_logger().warning(f'Annotate publish failed: {e}')

    def _run_ncnn_inference(self, frame_bgr, dx=0.0, dy=0.0):
        """Run NCNN inference on the input frame"""
        # Preprocessing - resize and normalize
        input_size = (self.imgsz, self.imgsz)
        img = cv2.resize(frame_bgr, input_size)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Normalize to [0, 1] and convert to NCNN Mat format
        img_norm = img_rgb.astype(np.float32) / 255.0

        # NCNN expects CHW format
        img_chw = np.transpose(img_norm, (2, 0, 1))

        # Create NCNN Mat from numpy array
        input_mat = ncnn.Mat(img_chw)

        # Create extractor
        ex = self.net.create_extractor()
        ex.set_light_mode(True)  # Optimize for mobile/edge devices
        ex.set_num_threads(4)

        # Run inference
        ex.input("in0", input_mat)
        ret, output_mat = ex.extract("out0")

        if ret != 0:
            self.get_logger().error(f'NCNN extraction failed with code: {ret}')
            return []

        # Convert output to numpy array
        output = np.array(output_mat)

        # Post-process detections
        return self._postprocess_detections(output, frame_bgr.shape, dx, dy)

    def _postprocess_detections(self, output, original_shape, dx=0.0, dy=0.0):
        """Post-process NCNN detection output"""
        h_orig, w_orig = original_shape[:2]
        scale_x = w_orig / self.imgsz
        scale_y = h_orig / self.imgsz

        detections = []

        if len(output.shape) == 2:
            predictions = output
        else:
            predictions = output.squeeze(0) if output.shape[0] == 1 else output[0]

        # Apply confidence filtering and NMS
        valid_detections = []

        for detection in predictions:
            if len(detection) >= 6:
                # Format: [x1, y1, x2, y2, conf, class] or [x_center, y_center, w, h, conf, class]
                # Check if this is center format (YOLO style) or corner format
                x1, y1, x2, y2, conf, cls = detection[:6]

                if conf < self.conf:
                    continue

                cls_id = int(cls)
                if cls_id != self.weed_class_id:
                    continue

                # Convert from model coordinates to original image coordinates
                # Assuming the model outputs are in corner format (x1, y1, x2, y2)
                mx = max(float(x1), float(y1), float(x2), float(y2))
                if mx <= 1.0 + 1e-6:
                    # normalized
                    x1_orig, y1_orig, x2_orig, y2_orig = x1 * w_orig, y1 * h_orig, x2 * w_orig, y2 * h_orig
                elif mx <= self.imgsz + 2.0:
                    # model pixel
                    x1_orig, y1_orig = x1 * scale_x, y1 * scale_y
                    x2_orig, y2_orig = x2 * scale_x, y2 * scale_y
                else:
                    # already original pixels
                    x1_orig, y1_orig, x2_orig, y2_orig = x1, y1, x2, y2

                # Check if this might be center format instead
                if x2_orig < x1_orig or y2_orig < y1_orig:
                    # Likely center format: convert from (cx, cy, w, h) to (x1, y1, x2, y2)
                    cx, cy, w, h = x1_orig, y1_orig, x2_orig, y2_orig
                    x1_orig = cx - w / 2
                    y1_orig = cy - h / 2
                    x2_orig = cx + w / 2
                    y2_orig = cy + h / 2

                # Apply motion compensation
                bx = (x1_orig + x2_orig) * 0.5 - dx
                by = (y1_orig + y2_orig) * 0.5 - dy
                wbox = max(1.0, x2_orig - x1_orig)
                hbox = max(1.0, y2_orig - y1_orig)

                nx1 = bx - 0.5 * wbox
                ny1 = by - 0.5 * hbox
                nx2 = bx + 0.5 * wbox
                ny2 = by + 0.5 * hbox

                valid_detections.append({
                    'box': [nx1, ny1, nx2, ny2],
                    'conf': float(conf),
                    'cls': cls_id
                })

        # Apply NMS if we have multiple detections
        if len(valid_detections) > 1:
            valid_detections = self._apply_nms(valid_detections, self.iou)

        return valid_detections

    def _apply_nms(self, detections, iou_threshold):
        """Apply Non-Maximum Suppression"""
        if len(detections) <= 1:
            return detections

        # Sort by confidence
        detections.sort(key=lambda x: x['conf'], reverse=True)

        keep = []
        while detections:
            current = detections.pop(0)
            keep.append(current)

            # Remove overlapping detections
            remaining = []
            for det in detections:
                if self._calculate_iou(current['box'], det['box']) < iou_threshold:
                    remaining.append(det)
            detections = remaining

        return keep

    def _calculate_iou(self, box1, box2):
        """Calculate Intersection over Union"""
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2

        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)

        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0

        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection

        return intersection / union if union > 0 else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = NCNNDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
