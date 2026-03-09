#!/usr/bin/env python3
"""
Phase 1 — Perception Node
Subscribes to RGB + aligned depth images from the RealSense D435.
Performs classical 2D object detection using HSV color segmentation.
Converts 2D detections to 3D poses in the camera frame.
Publishes geometry_msgs/PoseArray on /detected_poses.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
import numpy as np
import tf_transformations


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # ── Declare HSV threshold parameters (tunable via rqt_reconfigure) ──
        self.declare_parameter('h_low', 35)
        self.declare_parameter('h_high', 85)
        self.declare_parameter('s_low', 150)
        self.declare_parameter('s_high', 255)
        self.declare_parameter('v_low', 100)
        self.declare_parameter('v_high', 255)
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('max_depth_m', 2.0)

        # ── Camera intrinsics (populated from /camera/camera/color/camera_info) ──
        self.fx = None
        self.fy = None
        self.cx_cam = None
        self.cy_cam = None

        # ── CV Bridge ──
        self.bridge = CvBridge()

        # ── Publisher ──
        self.pose_pub = self.create_publisher(PoseArray, '/detected_poses', 10)
        self.debug_pub = self.create_publisher(Image, '/detection_overlay', 10)

        # ── Camera info subscriber ──
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback, 10
        )

        # ── Synchronized RGB + Depth subscribers ──
        rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info('Object Detector node started')

    def camera_info_callback(self, msg):
        """Extract camera intrinsic parameters from CameraInfo message."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx_cam = msg.k[2]
        self.cy_cam = msg.k[5]

    def synced_callback(self, rgb_msg, depth_msg):
        """Process synchronized RGB + depth frame pair."""
        if self.fx is None:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=5.0)
            return

        # ── Convert ROS messages to OpenCV arrays ──
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        # ── Get current HSV parameters ──
        h_low = self.get_parameter('h_low').value
        h_high = self.get_parameter('h_high').value
        s_low = self.get_parameter('s_low').value
        s_high = self.get_parameter('s_high').value
        v_low = self.get_parameter('v_low').value
        v_high = self.get_parameter('v_high').value
        min_area = self.get_parameter('min_contour_area').value
        max_depth = self.get_parameter('max_depth_m').value

        # ── Step 2a: BGR to HSV ──
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

        # ── Step 2b: Color segmentation (binary mask) ──
        mask = cv2.inRange(
            hsv,
            np.array([h_low, s_low, v_low]),
            np.array([h_high, s_high, v_high])
        )

        # ── Step 2c: Morphological cleanup ──
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Remove noise

        # ── Step 2d: Contour detection ──
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ── Build PoseArray ──
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'camera_color_optical_frame'

        debug_frame = rgb.copy()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            # ── Step 2e: Centroid via image moments ──
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # ── Step 2f: Orientation via second-order moments ──
            theta = 0.5 * np.arctan2(2 * M['mu11'], M['mu20'] - M['mu02'])

            # ── Step 3: Depth lookup + 3D conversion ──
            # Bounds check
            if cy >= depth.shape[0] or cx >= depth.shape[1]:
                continue

            depth_mm = depth[cy][cx]
            Z = float(depth_mm) * 0.001  # mm to meters

            if Z <= 0 or Z > max_depth:
                continue

            # Pinhole camera model: pixel to 3D
            X = (cx - self.cx_cam) * Z / self.fx
            Y = (cy - self.cy_cam) * Z / self.fy

            # Convert yaw angle to quaternion
            q = tf_transformations.quaternion_from_euler(0, 0, theta)

            # Build Pose message
            pose = Pose()
            pose.position.x = X
            pose.position.y = Y
            pose.position.z = Z
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            pose_array.poses.append(pose)

            # ── Debug visualization ──
            cv2.drawContours(debug_frame, [contour], -1, (0, 255, 0), 2)
            cv2.circle(debug_frame, (cx, cy), 5, (0, 0, 255), -1)
            # Draw orientation line
            line_len = 40
            ex = int(cx + line_len * np.cos(theta))
            ey = int(cy + line_len * np.sin(theta))
            cv2.line(debug_frame, (cx, cy), (ex, ey), (255, 0, 0), 2)
            cv2.putText(debug_frame, f'Z={Z:.2f}m', (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # ── Publish ──
        if pose_array.poses:
            self.pose_pub.publish(pose_array)
            self.get_logger().info(f'Detected {len(pose_array.poses)} objects')

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
