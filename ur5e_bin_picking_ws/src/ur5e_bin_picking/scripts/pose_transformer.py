#!/usr/bin/env python3
"""
Phase 2 — Coordinate Transformation Node
1. Broadcasts detected object poses as dynamic TF frames
2. Transforms poses from camera_color_optical_frame to base_link
3. Publishes transformed poses on /object_poses_base
"""
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs  # REQUIRED for PoseStamped transforms
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped


class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')

        # ── TF2 Buffer + Listener (reads all transforms in the system) ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── TF2 Broadcaster (publishes dynamic object frames) ──
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscribe to Phase 1 detections ──
        self.sub = self.create_subscription(
            PoseArray, '/detected_poses',
            self.detection_callback, 10
        )

        # ── Publish transformed poses for Phase 3 ──
        self.pub = self.create_publisher(PoseArray, '/object_poses_base', 10)

        self.get_logger().info('Pose Transformer node started')

    def detection_callback(self, msg):
        """
        For each detected object:
        1. Broadcast it as a dynamic TF frame (camera_color_optical_frame -> detected_object_N)
        2. Transform the pose from camera frame to base_link
        3. Publish all transformed poses
        """
        # ── Step 1: Broadcast each detection as a dynamic TF frame ──
        for i, pose in enumerate(msg.poses):
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'camera_color_optical_frame'
            t.child_frame_id = f'detected_object_{i}'
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w
            self.tf_broadcaster.sendTransform(t)

        # ── Step 2: Transform all poses to base_link frame ──
        result = PoseArray()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = 'base_link'

        for pose in msg.poses:
            ps = PoseStamped()
            ps.header = msg.header  # frame_id = camera_color_optical_frame
            ps.pose = pose

            try:
                transformed = self.tf_buffer.transform(
                    ps, 'base_link',
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                result.poses.append(transformed.pose)
            except tf2_ros.TransformException as ex:
                self.get_logger().error(f'Transform failed: {ex}')

        # ── Step 3: Publish transformed poses ──
        if result.poses:
            self.pub.publish(result)
            self.get_logger().info(
                f'Transformed {len(result.poses)} poses to base_link'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
