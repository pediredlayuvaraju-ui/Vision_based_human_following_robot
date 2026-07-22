import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from deep_sort_realtime.deepsort_tracker import DeepSort
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32
from ultralytics import YOLO


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            sensor_qos,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            sensor_qos,
        )

        self.lock_sub = self.create_subscription(
            Int32,
            "/lock_target_id",
            self.lock_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.model = YOLO("yolov8n.pt")

        self.tracker = DeepSort(
            max_age=45,
            n_init=2,
            max_cosine_distance=0.45,
            embedder_gpu=False,
            half=False,
            bgr=True,
        )

        # Target lock and re-identification
        self.locked_target_id = None
        self.target_features = None
        self.frames_since_seen = 0
        self.reid_score_threshold = 0.52

        # Robot speed
        self.linear_speed = 0.07
        self.max_angular_speed = 0.10
        self.search_speed = 0.035
        self.dead_zone_px = 45
        self.kp_ang = 0.20

        # Obstacle avoidance from LiDAR
        self.scan_ready = False
        self.front_min = float("inf")
        self.left_min = float("inf")
        self.right_min = float("inf")

        self.emergency_stop_distance = 0.35
        self.obstacle_avoid_distance = 0.85
        self.target_follow_distance = 0.95
        self.side_clearance = 0.35
        self.avoid_turn_speed = 0.10

        self.last_cmd_time = time.time()
        self.last_status_log_time = 0.0
        self.last_status_text = ""

        self.avoid_mode = False
        self.avoid_direction = 1
        self.avoid_start_time = 0.0

        self.get_logger().info("Human following node started with RE-ID + obstacle avoidance")

    def lock_callback(self, msg):
        if msg.data == -1:
            self.locked_target_id = None
            self.target_features = None
            self.frames_since_seen = 0
            self.stop_robot()
            self.get_logger().info("Target unlocked. Robot stopped.")
            return

        self.locked_target_id = int(msg.data)
        self.target_features = None
        self.frames_since_seen = 0
        self.stop_robot()
        self.get_logger().info(f"Locked target ID: {self.locked_target_id}. Waiting to capture features.")

    def scan_callback(self, msg):
        self.scan_ready = True
        self.front_min = self.get_sector_min(msg, 0, 40)
        self.left_min = self.get_sector_min(msg, 65, 60)
        self.right_min = self.get_sector_min(msg, -65, 60)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_sector_min(self, scan_msg, center_deg, width_deg):
        center = math.radians(center_deg)
        half_width = math.radians(width_deg / 2.0)

        vals = []
        for i, r in enumerate(scan_msg.ranges):
            if not math.isfinite(r):
                continue
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            angle = self.normalize_angle(angle)
            diff = abs(self.normalize_angle(angle - center))

            if diff <= half_width:
                vals.append(r)

        if len(vals) == 0:
            return float("inf")

        return float(min(vals))

    def track_id_to_int(self, track_id):
        try:
            return int(track_id)
        except Exception:
            return abs(hash(str(track_id))) % 10000

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(self.clamp(linear_x, -0.05, self.linear_speed))
        msg.angular.z = float(self.clamp(angular_z, -self.max_angular_speed, self.max_angular_speed))
        self.cmd_pub.publish(msg)
        self.last_cmd_time = time.time()
        self.last_status_log_time = 0.0
        self.last_status_text = ""

        self.avoid_mode = False
        self.avoid_direction = 1
        self.avoid_start_time = 0.0

    def search_target(self):
        if self.scan_ready and self.front_min < self.emergency_stop_distance:
            turn = self.avoid_turn_speed if self.left_min > self.right_min else -self.avoid_turn_speed
            self.publish_cmd(0.0, turn)
            return

        self.publish_cmd(0.0, self.search_speed)

    def log_status(self, status_text):
        now = time.time()
        if status_text != self.last_status_text or (now - self.last_status_log_time) > 1.0:
            self.get_logger().info(f"STATUS: {status_text}")
            self.last_status_text = status_text
            self.last_status_log_time = now

    def extract_features(self, frame, bbox):
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = map(int, bbox)

        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w - 1, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h - 1, y2))

        if x2 <= x1 or y2 <= y1:
            return None

        bw = x2 - x1
        bh = y2 - y1
        area = bw * bh
        aspect = bw / max(1, bh)

        # Torso region is more stable than full body
        tx1 = int(x1 + 0.20 * bw)
        tx2 = int(x1 + 0.80 * bw)
        ty1 = int(y1 + 0.18 * bh)
        ty2 = int(y1 + 0.70 * bh)

        roi = frame[ty1:ty2, tx1:tx2]
        if roi.size == 0:
            return None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        hist = cv2.calcHist(
            [hsv],
            [0, 1],
            None,
            [24, 24],
            [0, 180, 0, 256],
        )
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)

        rgb_mean = np.mean(roi.reshape(-1, 3), axis=0)

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        return {
            "hist": hist,
            "rgb_mean": rgb_mean,
            "area": float(area),
            "aspect": float(aspect),
            "center": np.array([cx, cy], dtype=np.float32),
        }

    def update_target_features(self, features):
        if features is None:
            return

        if self.target_features is None:
            self.target_features = features
            return

        alpha = 0.15

        self.target_features["hist"] = (
            (1.0 - alpha) * self.target_features["hist"] + alpha * features["hist"]
        )
        cv2.normalize(self.target_features["hist"], self.target_features["hist"], 0, 1, cv2.NORM_MINMAX)

        self.target_features["rgb_mean"] = (
            (1.0 - alpha) * self.target_features["rgb_mean"] + alpha * features["rgb_mean"]
        )

        self.target_features["area"] = (
            (1.0 - alpha) * self.target_features["area"] + alpha * features["area"]
        )

        self.target_features["aspect"] = (
            (1.0 - alpha) * self.target_features["aspect"] + alpha * features["aspect"]
        )

        self.target_features["center"] = features["center"]

    def reid_score(self, features):
        if self.target_features is None or features is None:
            return 0.0

        hist_score = cv2.compareHist(
            self.target_features["hist"],
            features["hist"],
            cv2.HISTCMP_CORREL,
        )
        hist_score = self.clamp(hist_score, 0.0, 1.0)

        rgb_dist = np.linalg.norm(self.target_features["rgb_mean"] - features["rgb_mean"])
        rgb_score = 1.0 - self.clamp(rgb_dist / 441.7, 0.0, 1.0)

        area_ratio = min(self.target_features["area"], features["area"]) / max(
            self.target_features["area"],
            features["area"],
            1.0,
        )

        aspect_diff = abs(self.target_features["aspect"] - features["aspect"])
        aspect_score = 1.0 - self.clamp(aspect_diff, 0.0, 1.0)

        score = 0.60 * hist_score + 0.25 * rgb_score + 0.15 * aspect_score

        if area_ratio < 0.20:
            score -= 0.20

        return float(self.clamp(score, 0.0, 1.0))

    def avoid_and_follow(self, bbox, frame_width, frame_height):
        x1, y1, x2, y2 = bbox

        bbox_w = max(1, x2 - x1)
        bbox_h = max(1, y2 - y1)

        bbox_h_ratio = bbox_h / max(1, frame_height)
        bbox_area_ratio = (bbox_w * bbox_h) / max(1, frame_width * frame_height)

        cx = (x1 + x2) / 2.0
        image_center = frame_width / 2.0
        error_px = cx - image_center

        target_centered = abs(error_px) < self.dead_zone_px * 1.5
        target_looks_close = bbox_h_ratio > 0.68 or bbox_area_ratio > 0.35

        # Target centering
        if abs(error_px) < self.dead_zone_px:
            target_angular = 0.0
            turn_status = "CENTERED"
        else:
            norm_error = error_px / image_center
            target_angular = -self.kp_ang * norm_error
            target_angular = self.clamp(target_angular, -self.max_angular_speed, self.max_angular_speed)

            if target_angular > 0.03:
                turn_status = "TURNING LEFT TO TARGET"
            elif target_angular < -0.03:
                turn_status = "TURNING RIGHT TO TARGET"
            else:
                turn_status = "CENTERED"

        if not self.scan_ready:
            self.publish_cmd(0.0, target_angular)
            return f"TARGET LOCKED | NO SCAN | {turn_status}"

        now = time.time()

        emergency_distance = 0.25
        obstacle_start = 1.10
        obstacle_clear = 1.25
        side_block = 0.45

        bypass_forward = 0.075
        bypass_turn = 0.16

        # Very close obstacle: rotate only
        if self.front_min < emergency_distance:
            self.avoid_mode = True
            self.avoid_direction = 1 if self.left_min > self.right_min else -1
            self.avoid_start_time = now

            side = "LEFT" if self.avoid_direction > 0 else "RIGHT"
            self.publish_cmd(0.0, bypass_turn * self.avoid_direction)
            return f"TARGET LOCKED | VERY CLOSE OBSTACLE | ROTATE {side}"

        # If target itself is close and centered, stop at safe distance
        if target_looks_close and target_centered and self.front_min < self.target_follow_distance:
            self.avoid_mode = False
            self.publish_cmd(0.0, target_angular)
            return "TARGET LOCKED | TARGET CLOSE | STOPPED AT SAFE DISTANCE"

        # If front is blocked but target is not close, treat it as obstacle and bypass
        if self.front_min < obstacle_start:
            if not self.avoid_mode:
                self.avoid_mode = True
                self.avoid_direction = 1 if self.left_min > self.right_min else -1
                self.avoid_start_time = now

        # Persistent bypass mode
        if self.avoid_mode:
            if self.front_min > obstacle_clear and (now - self.avoid_start_time) > 1.2:
                self.avoid_mode = False
            else:
                # Switch direction if chosen side becomes blocked
                if self.avoid_direction > 0 and self.left_min < side_block and self.right_min > self.left_min:
                    self.avoid_direction = -1
                elif self.avoid_direction < 0 and self.right_min < side_block and self.left_min > self.right_min:
                    self.avoid_direction = 1

                side = "LEFT" if self.avoid_direction > 0 else "RIGHT"
                self.publish_cmd(bypass_forward, bypass_turn * self.avoid_direction)
                return f"TARGET LOCKED | BYPASS OBSTACLE | ARC {side} + FORWARD"

        # Side obstacle only
        if self.right_min < side_block:
            self.publish_cmd(bypass_forward, 0.14)
            return "TARGET LOCKED | OBSTACLE RIGHT | FORWARD + LEFT"

        if self.left_min < side_block:
            self.publish_cmd(bypass_forward, -0.14)
            return "TARGET LOCKED | OBSTACLE LEFT | FORWARD + RIGHT"

        # Normal following
        if self.front_min > self.target_follow_distance:
            linear_x = self.linear_speed
            if turn_status == "CENTERED":
                move_status = "MOVING FORWARD"
            else:
                move_status = f"MOVING FORWARD + {turn_status}"
        else:
            linear_x = 0.0
            if turn_status == "CENTERED":
                move_status = "STOPPED AT SAFE DISTANCE"
            else:
                move_status = f"STOPPED + {turn_status}"

        self.publish_cmd(linear_x, target_angular)
        return f"TARGET LOCKED | {move_status}"

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        frame_h, frame_w = frame.shape[:2]

        results = self.model(frame, classes=[0], conf=0.15, imgsz=640, verbose=False)

        detections = []
        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                w = x2 - x1
                h = y2 - y1

                if w <= 10 or h <= 20:
                    continue

                detections.append(([float(x1), float(y1), float(w), float(h)], conf, "person"))

        tracks = self.tracker.update_tracks(detections, frame=frame)

        visible_tracks = []

        for track in tracks:
            if not track.is_confirmed():
                continue

            track_id = self.track_id_to_int(track.track_id)
            ltrb = track.to_ltrb()
            x1, y1, x2, y2 = map(int, ltrb)

            if x2 <= x1 or y2 <= y1:
                continue

            features = self.extract_features(frame, (x1, y1, x2, y2))

            visible_tracks.append(
                {
                    "id": track_id,
                    "bbox": (x1, y1, x2, y2),
                    "features": features,
                    "reid_score": self.reid_score(features),
                }
            )

        target_track = None
        best_reid_track = None
        best_reid_score = 0.0

        if self.locked_target_id is not None:
            for t in visible_tracks:
                if t["id"] == self.locked_target_id:
                    target_track = t
                    break

            # Exact ID lost: try appearance-based re-identification
            if target_track is None and self.target_features is not None:
                for t in visible_tracks:
                    score = t["reid_score"]
                    if score > best_reid_score:
                        best_reid_score = score
                        best_reid_track = t

                if best_reid_track is not None and best_reid_score >= self.reid_score_threshold:
                    old_id = self.locked_target_id
                    self.locked_target_id = best_reid_track["id"]
                    target_track = best_reid_track
                    self.get_logger().info(
                        f"RE-IDENTIFIED target: old ID {old_id} -> new ID {self.locked_target_id}, score={best_reid_score:.2f}"
                    )

        status_text = "NO TARGET LOCKED"

        if self.locked_target_id is None:
            self.search_target()
            status_text = "NO TARGET LOCKED | SEARCHING"

        elif target_track is not None:
            self.frames_since_seen = 0
            self.update_target_features(target_track["features"])
            status_text = self.avoid_and_follow(target_track["bbox"], frame_w, frame_h)

        else:
            self.frames_since_seen += 1
            self.search_target()
            status_text = f"TARGET LOCKED | TARGET LOST | SEARCHING ID {self.locked_target_id}"

        self.log_status(status_text)

        # Draw tracks
        for t in visible_tracks:
            x1, y1, x2, y2 = t["bbox"]
            tid = t["id"]

            if self.locked_target_id is None:
                label = f"ID {tid}"
                color = (255, 255, 0)
            elif tid == self.locked_target_id:
                label = f"TARGET ID {tid}"
                color = (0, 255, 0)
            elif target_track is not None and tid == target_track["id"]:
                label = f"RE-ID TARGET {tid}"
                color = (0, 255, 0)
            else:
                label = f"DISTRACTOR ID {tid} S:{t['reid_score']:.2f}"
                color = (0, 0, 255)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                color,
                1,
            )

        lidar_text = f"Front:{self.front_min:.2f} Left:{self.left_min:.2f} Right:{self.right_min:.2f}"
        cv2.putText(frame, status_text, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1)
        cv2.putText(frame, lidar_text, (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (255, 255, 255), 1)

        display_frame = cv2.resize(frame, (960, 540), interpolation=cv2.INTER_LINEAR)
        cv2.imshow("Human Following - ReID + Obstacle Avoidance", display_frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.stop_robot()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        time.sleep(0.2)
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
