#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .orientation_filter import ComplementaryFilter
from .point_cloud_accumulator import PointCloudAccumulator
from .ros_messages import make_imu_msg, make_pose_msg, make_transform, make_pointcloud_msg
from .scan_projection import scan_to_points

# --- Frame definitions for RVIZ ---
WORLD_FRAME = "odom"
BASE_FRAME = "base_link"
LASER_FRAME = "laser"

IDENTITY_ROTATION = (0.0, 0.0, 0.0, 1.0)


class FusionNode(Node):
    """Verbindet IMU und Lidar: Orientierung schätzen, Scans drehen, Punktwolke publizieren."""

    def __init__(self):
        # Creates a Node using the rclpy.node constructor, only provides node_name param
        super().__init__('fusion_calculation_node')

        """
        --- Creating subscribers ---
        Parameters:
            - msg_type, Nachrichtentyp, Klasse aus sensor_msgs.msg. Muss zum Publisher passen, sonst keine Verbindung
            - topic, Topic-Name. Führender / = absolut, Namespace wird ignoriert
            - callback, Wird pro Nachricht mit dieser als Argument aufgerufen. Referenz ohne (), also nur Verweis auf Funktion, kein Aufruf hier
            - qos_profile, Quality of Service. Zahl ist Kurzform für History KEEP_LAST mit Queue-Tiefe 10, Reliability RELIABLE
                (Queue-Tiefe 10 heißt: Kommt Callback nicht hinterher, werden bis zu 10 Nachrichten gepuffert, bei mehr fallen älteste weg.)
        """ 
        self.create_subscription(Imu, '/imu', self.six_dof_imu_orientation_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # slam_toolbox publiziert seine Pose (Frame map) erst nach erfolgreich verarbeitetem Scan
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.slam_pose_callback, 10)

        # --- Publishers for RViz ---
        self.six_dof_imu_fused_pub = self.create_publisher(Imu, '/imu_fused', 10)
        self.six_dof_imu_pose_pub = self.create_publisher(PoseStamped, '/imu_pose', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/pointcloud', 10)

        # Publishes IMU transform on every IMU-message. 
        # Without, RViz/slam_toolbox dont know devices position
        self.tf_broadcaster = TransformBroadcaster(self)

        # Used once to publish the connection between laser and base frame so that 
        # the relative position of the laser to the imu can be defined only once 
        # and does not need to be recalculated for every IMU message.
        self.static_broadcaster = StaticTransformBroadcaster(self)  
        self._publish_static_lidar_tf()

        # Calculates orientation using gyro and accelerometer. 
        # Gyro counts alpha = 90 %, accelerometer 10 %.
        self.six_dof_imu_orientation_filter = ComplementaryFilter(alpha=0.9)

        # Edgelength of a vocel in meter. 
        # Used for Voxel Grid Downsampling representation of the LIDARs pointcloud.
        voxel_size = self.declare_parameter('voxel_size', 0.05).value
        self.point_cloud = PointCloudAccumulator(voxel_size)

        # Stores time delta
        self.six_dof_imu_prev_time = None
        # Stores current orientation. 
        # IDENTITY_ROTATION ist starting value and basicaly means "no rotation".
        self.six_dof_imu_orientation = IDENTITY_ROTATION
        self.slam_position = (0.0, 0.0, 0.0)

    
    def _now(self):
        """Returns the current time."""
        return self.get_clock().now().to_msg()

    
    def _compute_six_dof_imu_dt(self):
        """Calculates the time delta"""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.01 if self.six_dof_imu_prev_time is None else current_time - self.six_dof_imu_prev_time
        self.six_dof_imu_prev_time = current_time
        return dt

    
    def _publish_static_lidar_tf(self):
        """Publishes the lidars transform"""
        # TBD: Lidar sitzt aktuell genau auf der IMU (keine Verschiebung, keine Drehung)
        t = make_transform(self._now(), BASE_FRAME, LASER_FRAME, (0.0, 0.0, 0.0), IDENTITY_ROTATION)
        self.static_broadcaster.sendTransform(t)

    def six_dof_imu_orientation_callback(self, msg: Imu):
        """Uses current gyroscope and acceleration values to calculate current IMU orrientation.
        Publishes the IMU orientation in the World and Base frame.
        Broadcasts the current orientation using the tf_broadcaster so rviz ans slam_toolbox can see it.
        
        Args:
            msg: Imu from /imu topic. Message from the IMU node.

        Returns:
            None
        """
        six_dof_imu_gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        six_dof_imu_accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.six_dof_imu_orientation = self.six_dof_imu_orientation_filter.update(six_dof_imu_gyro, six_dof_imu_accel, self._compute_six_dof_imu_dt())

        stamp = self._now()
        self.six_dof_imu_fused_pub.publish(make_imu_msg(stamp, WORLD_FRAME, msg, self.six_dof_imu_orientation))
        self.six_dof_imu_pose_pub.publish(make_pose_msg(stamp, BASE_FRAME, self.six_dof_imu_orientation))
        self.tf_broadcaster.sendTransform(
            make_transform(stamp, WORLD_FRAME, BASE_FRAME, self.slam_position, self.six_dof_imu_orientation))

    def lidar_callback(self, msg: LaserScan):
        """Adds the LIDARs LaserScan into the Voxel Grid/Pointcloud
        Publishes the grid/pointcloud.

        Args:
            msg: LaserScan from /scan LIDAR topic. 
                LaserScan is one 360 degree LIDAR turn, including all its scanned points.

        Returns:
            None
        """
        self.point_cloud.add(scan_to_points(msg, self.six_dof_imu_orientation, self.slam_position))
        self.pointcloud_pub.publish(
            make_pointcloud_msg(self._now(), WORLD_FRAME, self.point_cloud.points)
            )

    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Stores the device position estimated by slam_toolbox.
        Unfortuantely a circular dependency exists between the imu orientation and slam position.
        Slam_toolbox uses the IMUs base_link frame for estimating the position.
        The IMU callback creates the base_link transform using the slam_position.
        
        Args:
            - msg:  Pose from /pose in frame map. Only the position is used,
                    the orientation comes from the IMU filter.
        """
        p = msg.pose.pose.position
        self.slam_position = (p.x, p.y, p.z)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
