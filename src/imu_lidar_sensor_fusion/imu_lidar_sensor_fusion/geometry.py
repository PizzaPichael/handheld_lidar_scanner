import math

# Quaternionen als Tupel (x, y, z, w), wie in ROS-Nachrichten


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz
    )


def rotate_vector(v, q):
    # v' = q * (v, 0) * q_conj
    qx, qy, qz, qw = q
    q_conj = (-qx, -qy, -qz, qw)
    v_quat = (v[0], v[1], v[2], 0.0)
    res = quaternion_multiply(quaternion_multiply(q, v_quat), q_conj)
    return (res[0], res[1], res[2])


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw/2)
    sy = math.sin(yaw/2)
    cp = math.cos(pitch/2)
    sp = math.sin(pitch/2)
    cr = math.cos(roll/2)
    sr = math.sin(roll/2)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw
