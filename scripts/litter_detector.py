#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class LitterDetector(Node):
    def __init__(self):
        super().__init__('litter_detector')

        self.bridge = CvBridge()

        # Camera intrinsic matrix parameters (defaults if camera_info is delayed)
        self.fx = 554.25
        self.fy = 554.25
        self.cx = 320.0
        self.cy = 240.0
        self.have_camera_info = False

        self.latest_depth_image = None
        self.latest_depth_time = None

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Color filter definitions for different litter types (HSV)
        self.litter_classes = {
            'soda_can_red': {
                'ranges': [
                    (np.array([0, 100, 60]), np.array([10, 255, 255])),
                    (np.array([170, 100, 60]), np.array([180, 255, 255]))
                ],
                'color_rgba': (1.0, 0.1, 0.1, 0.85),
                'shape': Marker.CYLINDER
            },
            'soda_can_blue': {
                'ranges': [
                    (np.array([100, 120, 60]), np.array([130, 255, 255]))
                ],
                'color_rgba': (0.1, 0.2, 1.0, 0.85),
                'shape': Marker.CYLINDER
            },
            'litter_1': {  # Green box
                'ranges': [
                    (np.array([40, 100, 60]), np.array([80, 255, 255]))
                ],
                'color_rgba': (0.1, 0.9, 0.1, 0.85),
                'shape': Marker.CUBE
            },
            'litter_2': {  # Yellow / Orange box
                'ranges': [
                    (np.array([18, 120, 100]), np.array([35, 255, 255]))
                ],
                'color_rgba': (1.0, 0.8, 0.1, 0.85),
                'shape': Marker.CUBE
            },
            'litter_3': {  # Purple box
                'ranges': [
                    (np.array([135, 100, 60]), np.array([169, 255, 255]))
                ],
                'color_rgba': (0.8, 0.2, 0.7, 0.85),
                'shape': Marker.CUBE
            }
        }

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_cb, 10)
        self.create_subscription(Image, '/camera/image', self.rgb_cb, 10)
        self.create_subscription(Image, '/camera/depth_image', self.depth_cb, 10)

        # Publishers
        self.detected_pose_pub = self.create_publisher(PoseStamped, '/detected_litter', 10)
        self.relative_pose_pub = self.create_publisher(PoseStamped, '/detected_litter_relative', 10)
        self.detected_name_pub = self.create_publisher(String, '/detected_litter_name', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/litter_markers', 10)
        self.debug_image_pub = self.create_publisher(Image, '/camera/detection_image', 10)

        # Cache of detected targets: {model_name: {'pose_map': PoseStamped, 'pose_base': PoseStamped, 'last_seen': time}}
        self.detected_targets = {}

        self.get_logger().info("LitterDetector initialized and waiting for camera stream...")

    def camera_info_cb(self, msg: CameraInfo):
        if not self.have_camera_info:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.have_camera_info = True
            self.get_logger().info(f"Camera intrinsics received: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")

    def depth_cb(self, msg: Image):
        try:
            # Handle float32 depth
            if msg.encoding in ['32FC1', '32F']:
                self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                self.latest_depth_image = depth_raw.astype(np.float32) / 1000.0  # mm to meters if 16UC1
            self.latest_depth_time = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f"Error converting depth image: {e}")

    def rgb_cb(self, msg: Image):
        if self.latest_depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Error converting RGB image: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        debug_img = cv_image.copy()

        now = self.get_clock().now()
        marker_array = MarkerArray()
        marker_id = 0

        for name, info in self.litter_classes.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lower, upper) in info['ranges']:
                curr_mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, curr_mask)

            # Morphological cleaning
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 160:  # Ignore tiny noise artifacts
                    continue

                # Bounding box & centroid
                x, y, w, h = cv2.boundingRect(cnt)
                u = int(x + w / 2)
                v = int(y + h / 2)

                # Sample depth from a 5x5 patch around the centroid
                v_min = max(0, v - 2)
                v_max = min(self.latest_depth_image.shape[0], v + 3)
                u_min = max(0, u - 2)
                u_max = min(self.latest_depth_image.shape[1], u + 3)
                depth_patch = self.latest_depth_image[v_min:v_max, u_min:u_max]
                valid_depths = depth_patch[np.isfinite(depth_patch) & (depth_patch > 0.15) & (depth_patch < 6.0)]

                if len(valid_depths) == 0:
                    continue

                z_c = float(np.median(valid_depths))
                x_c = float((u - self.cx) * z_c / self.fx)
                y_c = float((v - self.cy) * z_c / self.fy)

                # Draw bounding box and label on debug visual
                cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(debug_img, (u, v), 4, (0, 0, 255), -1)
                label = f"{name}: {z_c:.2f}m"
                cv2.putText(debug_img, label, (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

                # Construct PoseStamped in camera optical frame
                pose_cam = PoseStamped()
                pose_cam.header.stamp = msg.header.stamp
                pose_cam.header.frame_id = 'camera_optical_frame'
                pose_cam.pose.position.x = x_c
                pose_cam.pose.position.y = y_c
                pose_cam.pose.position.z = z_c
                pose_cam.pose.orientation.w = 1.0

                # Transform to base_footprint and map frames
                try:
                    # Transform to base_footprint
                    tf_base = self.tf_buffer.lookup_transform(
                        'base_footprint',
                        'camera_optical_frame',
                        rclpy.time.Time()
                    )
                    pose_base = do_transform_pose(pose_cam.pose, tf_base)
                    stamped_base = PoseStamped()
                    stamped_base.header.frame_id = 'base_footprint'
                    stamped_base.header.stamp = now.to_msg()
                    stamped_base.pose = pose_base

                    # Transform to map
                    tf_map = self.tf_buffer.lookup_transform(
                        'map',
                        'camera_optical_frame',
                        rclpy.time.Time()
                    )
                    pose_map = do_transform_pose(pose_cam.pose, tf_map)
                    stamped_map = PoseStamped()
                    stamped_map.header.frame_id = 'map'
                    stamped_map.header.stamp = now.to_msg()
                    stamped_map.pose = pose_map

                    # Set custom frame or topic identification (encoded in header frame_id or separate topic)
                    # We store the target
                    self.detected_targets[name] = {
                        'name': name,
                        'pose_map': stamped_map,
                        'pose_base': stamped_base,
                        'last_seen': now
                    }

                    # Publish detections
                    self.detected_pose_pub.publish(stamped_map)
                    self.relative_pose_pub.publish(stamped_base)
                    self.detected_name_pub.publish(String(data=name))

                    # Create RViz 3D Marker
                    marker = Marker()
                    marker.header = stamped_map.header
                    marker.ns = "litter_targets"
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = info['shape']
                    marker.action = Marker.ADD
                    marker.pose = stamped_map.pose
                    if info['shape'] == Marker.CYLINDER:
                        marker.scale.x = 0.08
                        marker.scale.y = 0.08
                        marker.scale.z = 0.12
                    else:
                        marker.scale.x = 0.08
                        marker.scale.y = 0.08
                        marker.scale.z = 0.08

                    r, g, b, a = info['color_rgba']
                    marker.color.r = r
                    marker.color.g = g
                    marker.color.b = b
                    marker.color.a = a
                    marker.lifetime.sec = 2
                    marker_array.markers.append(marker)

                    # Text Marker
                    text_marker = Marker()
                    text_marker.header = stamped_map.header
                    text_marker.ns = "litter_labels"
                    text_marker.id = marker_id
                    marker_id += 1
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.pose = stamped_map.pose
                    text_marker.pose.position.z += 0.18
                    text_marker.text = f"{name} ({z_c:.2f}m)"
                    text_marker.scale.z = 0.10
                    text_marker.color.r = 1.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 1.0
                    text_marker.color.a = 1.0
                    text_marker.lifetime.sec = 2
                    marker_array.markers.append(text_marker)

                except Exception as ex:
                    # TF might not be ready in the first few ticks
                    pass

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

        # Publish debug visualization image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LitterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
