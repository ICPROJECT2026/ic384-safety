"""RGB-D person detection, with a separate input to the latched supervisor."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

from .person_safety_core import PersonStopPolicy, nearest_depth, pair_is_fresh


class PersonSafety(Node):
    def __init__(self):
        super().__init__('person_safety')
        defaults = {
            'model_path': '', 'device': 'cpu', 'confidence': 0.35,
            'stop_distance': 1.0, 'release_distance': 1.5,
            'frame_timeout': 0.3, 'sync_tolerance': 0.05,
            'aligned_depth_confirmed': False, 'depth_scale': 0.001,
            'depth_min': 0.6, 'depth_max': 8.0,
            'rgb_topic': '/camera/rgb/image_raw',
            'depth_topic': '/camera/aligned_depth/image_raw',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        # Configuration is fixed at startup; restart after changing parameters.
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        self.policy = PersonStopPolicy(self.cfg['stop_distance'], self.cfg['release_distance'])
        for key in ('frame_timeout', 'depth_scale', 'depth_min', 'depth_max'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError('invalid '+key)
        if (not math.isfinite(self.cfg['sync_tolerance']) or self.cfg['sync_tolerance'] < 0
                or not math.isfinite(self.cfg['confidence']) or not 0 < self.cfg['confidence'] < 1
                or self.cfg['depth_min'] >= self.cfg['depth_max']):
            raise ValueError('invalid synchronization, confidence or depth range')
        self.clear_pub = self.create_publisher(Bool, '/safety/input/person_clear', 1)
        self.status_pub = self.create_publisher(String, '/safety/person_status', 1)
        self.bridge = CvBridge()
        self.rgb = self.depth = None
        self.last_pair = None
        self.last_result = None
        self.model = None
        self.model_error = ''
        self.publish(self.policy.fault('initializing'))
        try:
            model_path = Path(self.cfg['model_path']).expanduser()
            if not self.cfg['model_path'] or not model_path.is_file() or model_path.suffix != '.pt':
                raise ValueError('model_path must be an existing local YOLO .pt detection model')
            from ultralytics import YOLO
            self.model = YOLO(str(model_path.resolve()), task='detect')
            self.person_ids = [int(k) for k, v in self.model.names.items() if str(v).lower() == 'person']
            if not self.person_ids or self.model.task != 'detect':
                raise ValueError('model must detect a class named person')
        except Exception as exc:
            self.model = None
            self.model_error = 'detector unavailable: '+str(exc)
            self.get_logger().error(self.model_error)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, kind in ((self.cfg['rgb_topic'], 'rgb'), (self.cfg['depth_topic'], 'depth')):
            self.create_subscription(Image, topic, lambda msg, k=kind: self.receive(k, msg), qos)
        self.create_timer(0.05, self.tick)

    def receive(self, kind, msg):
        setattr(self, kind, (msg, time.monotonic()))

    def publish(self, decision):
        self.clear_pub.publish(Bool(data=decision.clear))
        self.status_pub.publish(String(data=json.dumps({
            'clear': decision.clear, 'reason': decision.reason,
            'nearest_optical_depth_m': decision.nearest_m,
        }, allow_nan=False)))

    def fresh(self, stamps, received):
        return pair_is_fresh(stamps, received,
                             self.get_clock().now().nanoseconds*1e-9, time.monotonic(),
                             self.cfg['frame_timeout'], self.cfg['sync_tolerance'])

    def tick(self):
        try:
            self.process()
        except Exception as exc:
            self.publish(self.policy.fault('processing error: '+str(exc)))

    def process(self):
        if self.model is None:
            self.publish(self.policy.fault(self.model_error))
            return
        if not self.cfg['aligned_depth_confirmed']:
            self.publish(self.policy.fault('RGB/depth alignment not confirmed'))
            return
        if self.rgb is None or self.depth is None:
            self.publish(self.policy.fault('waiting for RGB and depth'))
            return
        rgb, rgb_received = self.rgb
        depth, depth_received = self.depth
        stamps = tuple(m.header.stamp.sec + m.header.stamp.nanosec*1e-9 for m in (rgb, depth))
        received = (rgb_received, depth_received)
        if not self.fresh(stamps, received):
            self.publish(self.policy.fault('frames stale, future-dated or unsynchronized'))
            return
        # Never turn repeated/frozen images into new clearance heartbeats.
        if self.last_pair is not None and any(a <= b for a, b in zip(stamps, self.last_pair)):
            if min(stamps) < min(self.last_pair):
                self.last_pair = None  # permit recovery after a simulation clock reset
                self.publish(self.policy.fault('camera timestamp moved backwards'))
            elif self.last_result is None or time.monotonic()-self.last_result > self.cfg['frame_timeout']:
                self.publish(self.policy.fault('no new frame pair'))
            return
        self.last_pair = stamps
        if (not rgb.header.frame_id or rgb.header.frame_id != depth.header.frame_id
                or (rgb.width, rgb.height) != (depth.width, depth.height)
                or min(rgb.width, rgb.height) <= 0):
            self.publish(self.policy.fault('RGB/depth geometry or optical frame mismatch'))
            return
        if depth.encoding not in ('16UC1', '32FC1'):
            self.publish(self.policy.fault('unsupported depth encoding'))
            return
        image = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        zmap = self.bridge.imgmsg_to_cv2(depth, 'passthrough')
        if image.shape != (rgb.height, rgb.width, 3) or zmap.shape != (rgb.height, rgb.width):
            self.publish(self.policy.fault('decoded image dimensions do not match metadata'))
            return
        scale = self.cfg['depth_scale'] if depth.encoding == '16UC1' else 1.0
        # A dead depth stream must not look clear just because detection found no person.
        if nearest_depth(zmap.flat, scale, self.cfg['depth_min'], self.cfg['depth_max']) is None:
            self.publish(self.policy.fault('depth image invalid'))
            return
        results = self.model.predict(image, classes=self.person_ids, conf=self.cfg['confidence'],
                                     device=self.cfg['device'], verbose=False)
        if len(results) != 1 or results[0].boxes is None:
            raise ValueError('detector returned no valid detection result')
        boxes = results[0].boxes.xyxy.cpu().tolist()
        distances = []
        for box in boxes:
            if len(box) != 4 or not all(math.isfinite(v) for v in box):
                raise ValueError('invalid person bounding box')
            x1, y1, x2, y2 = box
            if not 0 <= x1 < x2 <= rgb.width or not 0 <= y1 < y2 <= rgb.height:
                raise ValueError('person bounding box outside image')
            roi = zmap[math.floor(y1):math.ceil(y2), math.floor(x1):math.ceil(x2)]
            distances.append(nearest_depth(roi.flat, scale, self.cfg['depth_min'], self.cfg['depth_max']))
        # Inference time counts toward freshness, including a stalled model.
        if not self.fresh(stamps, received):
            self.publish(self.policy.fault('inference result expired'))
            return
        decision = self.policy.evaluate(distances)
        self.last_result = time.monotonic()
        self.publish(decision)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonSafety()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish(node.policy.fault('node shutting down'))
            node.destroy_node()
        rclpy.shutdown()
