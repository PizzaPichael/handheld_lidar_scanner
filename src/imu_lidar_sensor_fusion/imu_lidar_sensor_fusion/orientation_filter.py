import math

from .geometry import euler_to_quaternion


class ComplementaryFilter:
    """Schätzt die Orientierung aus Gyroskop und Accelerometer.

    Roll und Pitch: Gyro-Integration, korrigiert durch die Neigung aus dem Accelerometer.
    Yaw: nur Gyro-Integration (kein Magnetometer), driftet daher mit der Zeit.
    """

    def __init__(self, alpha=0.9):
        self.alpha = alpha  # Bias, wie sehr der Algorithmus den Gyro-Daten vertraut
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(self, gyro_deg_s, accel, dt):
        """gyro_deg_s: (gx, gy, gz) in deg/s, accel: (ax, ay, az) in m/s², dt in s.

        Gibt die neue Orientierung als Quaternion (x, y, z, w) zurück.
        """
        # Gyroskop Winkeländerung, Umrechnung von deg/s in rad/s
        gx, gy, gz = (math.radians(g) for g in gyro_deg_s)
        self.roll += gx * dt
        self.pitch += gy * dt
        self.yaw += gz * dt

        # Accelerometer Neigung
        ax, ay, az = accel
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        self.roll = self.alpha * self.roll + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc

        return euler_to_quaternion(self.roll, self.pitch, self.yaw)
