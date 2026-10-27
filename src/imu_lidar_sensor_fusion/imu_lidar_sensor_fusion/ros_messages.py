from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg

# Hilfsfunktionen zum Bauen von ROS-Nachrichten. Quaternionen als Tupel (x, y, z, w).

COVARIANCE = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]


def _set_quaternion(target, q):
    target.x, target.y, target.z, target.w = q


def make_imu_msg(stamp, frame_id, source: Imu, orientation):
    """Kopiert die Rohdaten aus source und ergänzt die berechnete Orientierung."""
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    _set_quaternion(msg.orientation, orientation)
    msg.angular_velocity = source.angular_velocity
    msg.linear_acceleration = source.linear_acceleration
    msg.orientation_covariance = COVARIANCE
    msg.angular_velocity_covariance = COVARIANCE
    msg.linear_acceleration_covariance = COVARIANCE
    return msg


def make_pose_msg(stamp, frame_id, orientation):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    _set_quaternion(msg.pose.orientation, orientation)
    return msg


def make_transform(stamp, parent_frame, child_frame, translation, rotation):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = translation
    _set_quaternion(t.transform.rotation, rotation)
    return t


def make_pointcloud_msg(stamp, frame_id, points):
    header = std_msgs.msg.Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return pc2.create_cloud_xyz32(header, points)
