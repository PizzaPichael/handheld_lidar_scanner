#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .orientation_filter import ComplementaryFilter
from .point_cloud_accumulator import PointCloudAccumulator
from .ros_messages import make_imu_msg, make_pose_msg, make_transform, make_pointcloud_msg
from .scan_projection import scan_to_points

WORLD_FRAME = "odom"
BASE_FRAME = "base_link"
LASER_FRAME = "laser"

IDENTITY_ROTATION = (0.0, 0.0, 0.0, 1.0)


class FusionNode(Node):
    """Verbindet IMU und Lidar: Orientierung schätzen, Scans drehen, Punktwolke publizieren."""

    def __init__(self):
        super().__init__('fusion_calculation_node')

        # Subscriber
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(PoseStamped, '/slam_toolbox/pose', self.slam_pose_callback, 10)

        # Publisher für RViz
        self.imu_fused_pub = self.create_publisher(Imu, '/imu_fused', 10)
        self.imu_pose_pub = self.create_publisher(PoseStamped, '/imu_pose', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/pointcloud', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_lidar_tf()

        self.orientation_filter = ComplementaryFilter(alpha=0.9)
        voxel_size = self.declare_parameter('voxel_size', 0.05).value  # Kantenlänge eines Voxels in m
        self.point_cloud = PointCloudAccumulator(voxel_size)

        self.prev_time = None
        self.orientation = IDENTITY_ROTATION
        self.slam_position = (0.0, 0.0, 0.0)

    def _now(self):
        return self.get_clock().now().to_msg()

    def _compute_dt(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.01 if self.prev_time is None else current_time - self.prev_time
        self.prev_time = current_time
        return dt

    def _publish_static_lidar_tf(self):
        # Lidar sitzt aktuell genau auf der IMU (keine Verschiebung, keine Drehung)
        t = make_transform(self._now(), BASE_FRAME, LASER_FRAME, (0.0, 0.0, 0.0), IDENTITY_ROTATION)
        self.static_broadcaster.sendTransform(t)

    def imu_callback(self, msg: Imu):
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.orientation = self.orientation_filter.update(gyro, accel, self._compute_dt())

        stamp = self._now()
        self.imu_fused_pub.publish(make_imu_msg(stamp, WORLD_FRAME, msg, self.orientation))
        self.imu_pose_pub.publish(make_pose_msg(stamp, BASE_FRAME, self.orientation))
        self.tf_broadcaster.sendTransform(
            make_transform(stamp, WORLD_FRAME, BASE_FRAME, self.slam_position, self.orientation))

    def lidar_callback(self, msg: LaserScan):
        self.point_cloud.add(scan_to_points(msg, self.orientation, self.slam_position))
        self.pointcloud_pub.publish(make_pointcloud_msg(self._now(), WORLD_FRAME, self.point_cloud.points))

    def slam_pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        self.slam_position = (p.x, p.y, p.z)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
