#!/usr/bin/env python3
import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_calculation_node')

        # Subscriber
        self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.slam_pose_sub = self.create_subscription(PoseStamped, '/slam_toolbox/pose', self.slam_pose_callback, 10)

        # Publisher für RViz
        self.imu_fused_pub = self.create_publisher(Imu, '/imu_fused', 10)
        self.imu_marker_pub = self.create_publisher(Marker, '/imu_marker', 10)
        self.imu_pose_pub = self.create_publisher(PoseStamped, '/imu_pose', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_lidar_tf()

        self.prev_time = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = 0.9 # Bias, wie sehr der Algorithmus den IMU Daten vertraut

        self.velocity = [0.0, 0.0, 0.0]   # vx, vy, vz (m/s)
        self.position = [0.0, 0.0, 0.0]   # x, y, z (m)
        self.acc_bias = [0.0, 0.0, 0.0]   # optionale Bias-Initialisierung
        self.gravity = 9.80665
        self.acc_threshold = 0.02  # m/s^2; Messerärmung unter diesem Wert wird als 0 behandelt
        
        self.slam_position = [0.0, 0.0, 0.0]

    def rotate_vector_by_quaternion(self, v, qx, qy, qz, qw):
        # v: (x,y,z)
        # q = (qx,qy,qz,qw)
        # Implementierung: v' = q * (0, v) * q_conj
        # Schrittweise (kein externes tf nötig)
        # Quaternion multiply helper
        def q_mult(a, b):
            ax, ay, az, aw = a
            bx, by, bz, bw = b
            return (
                aw*bx + ax*bw + ay*bz - az*by,
                aw*by - ax*bz + ay*bw + az*bx,
                aw*bz + ax*by - ay*bx + az*bw,
                aw*bw - ax*bx - ay*by - az*bz
            )
        q = (qx, qy, qz, qw)
        q_conj = (-qx, -qy, -qz, qw)
        v_quat = (v[0], v[1], v[2], 0.0)
        tmp = q_mult(q, v_quat)
        res = q_mult(tmp, q_conj)
        return (res[0], res[1], res[2])

    def _compute_dt(self):
        # Zeitdifferenz
        current_time = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.01 if self.prev_time is None else current_time - self.prev_time
        self.prev_time = current_time
        return dt

    def _update_orientation(self, msg: Imu, dt):
        # Gyroskop Winkeländerung
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # Umrechnung von deg/s in rad/s
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        self.roll += gx * dt
        self.pitch += gy * dt
        self.yaw += gz * dt

        # Accelerometer Neigung
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        # Complementary Filter
        self.roll = self.alpha * self.roll + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc

        # Quaternion
        cy = math.cos(self.yaw/2)
        sy = math.sin(self.yaw/2)
        cp = math.cos(self.pitch/2)
        sp = math.sin(self.pitch/2)
        cr = math.cos(self.roll/2)
        sr = math.sin(self.roll/2)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw

    def _update_translation(self, msg: Imu, quat, dt):
        #----Translation---
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        # Bias korrigieren (falls du Bias geschätzt hast)
        ax -= self.acc_bias[0]
        ay -= self.acc_bias[1]
        az -= self.acc_bias[2]

        # Beschleunigungsvektor im Body-frame
        acc_body = (ax, ay, az)

        # In Weltframe rotieren
        qx, qy, qz, qw = quat
        acc_world = self.rotate_vector_by_quaternion(acc_body, qx, qy, qz, qw)

        # Gravitation abziehen (angenommen +Z zeigt "up" im Weltframe)
        ax_w, ay_w, az_w = acc_world
        az_w -= self.gravity

        # Rauschunterdrückung / Dämpfung kleiner Werte
        if abs(ax_w) < self.acc_threshold: ax_w = 0.0
        if abs(ay_w) < self.acc_threshold: ay_w = 0.0
        if abs(az_w) < self.acc_threshold: az_w = 0.0

        # Integration (velocity und position)
        self.velocity[0] += ax_w * dt
        self.velocity[1] += ay_w * dt
        self.velocity[2] += az_w * dt

        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt

        # Optional: sehr einfache Geschwindigkeitsdrift-Dämpfung (kleine Werte auf 0 setzen)
        vel_thresh = 0.001
        for i in range(3):
            if abs(self.velocity[i]) < vel_thresh:
                self.velocity[i] = 0.0

    def _publish_imu_fused(self, msg: Imu, quat):
        qx, qy, qz, qw = quat
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "odom"
        # Orientation (Quaternion)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        # Kopiere ursprüngliche Sensordaten
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.linear_acceleration = msg.linear_acceleration
        # Setze Kovarianz-Matrizen
        imu_msg.orientation_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        imu_msg.angular_velocity_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        imu_msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        self.imu_fused_pub.publish(imu_msg)

    def _publish_imu_pose(self, quat):
        qx, qy, qz, qw = quat
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.imu_pose_pub.publish(pose_msg)

    def _publish_imu_marker(self, quat):
        qx, qy, qz, qw = quat
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.scale.x = 0.1 # Länge der X-Achse
        marker.scale.y = 0.1  # Länge der Y-Achse
        marker.scale.z = 0.1  # Länge der Z-Achse
        
        # Marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # Orientierung des Pfeils setzen
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        # Marker Position setzen (ersetze oder ergänze dein marker.pose)
        marker.pose.position.x = self.slam_position[0]
        marker.pose.position.y = self.slam_position[1]
        marker.pose.position.z = self.slam_position[2]
        self.imu_marker_pub.publish(marker)

    def _publish_transformation(self, quat):
        qx, qy, qz, qw = quat
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"          # Weltframe
        t.child_frame_id = "base_link"       # laser-Frame
        t.transform.translation.x = self.slam_position[0]
        t.transform.translation.y = self.slam_position[1]
        t.transform.translation.z = self.slam_position[2]
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def _publish_static_lidar_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser"
        t.transform.translation.x = 0.0 # 0.1 = Beispiel, 10 cm vor IMU
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)


    def slam_pose_callback(self, msg: PoseStamped):
        self.slam_position[0] = msg.pose.position.x
        self.slam_position[1] = msg.pose.position.y
        self.slam_position[2] = msg.pose.position.z

    def imu_callback(self, msg: Imu):
        dt = self._compute_dt()
        quaternion = self._update_orientation(msg, dt)
        #self._update_translation(msg, quaternion, dt)
        self._publish_imu_fused(msg, quaternion)
        self._publish_imu_pose(quaternion)        
        self._publish_imu_marker(quaternion)
        self._publish_transformation(quaternion)
        
        # Logging
        qx, qy, qz, qw = quaternion
        self.get_logger().info(
            f"Quaternion: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}"
        )

    def lidar_callback(self, msg: LaserScan):
        msg.header.frame_id = "laser"
        self.get_logger().info(
            f"Lidar - min={msg.range_min:.2f}, max={msg.range_max:.2f}, ranges[0]={msg.ranges[0]:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
