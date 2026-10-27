import math

from .geometry import rotate_vector


def scan_to_points(scan, orientation, position):
    """Wandelt einen LaserScan in 3D-Punkte im Weltframe.

    Jeder Punkt wird um die Orientierung (x, y, z, w) des Scanners gedreht und dann um
    dessen Position (x, y, z) verschoben. Messwerte außerhalb von range_min/range_max
    werden verworfen.
    """
    px, py, pz = position
    angle = scan.angle_min
    points = []
    for r in scan.ranges:
        if scan.range_min < r < scan.range_max:
            x, y, z = rotate_vector((r * math.cos(angle), r * math.sin(angle), 0.0), orientation)
            points.append((x + px, y + py, z + pz))
        angle += scan.angle_increment
    return points


# WIP, noch nicht verwendet: soll den Scan auf einen Winkelbereich nach vorne (x-Achse) begrenzen.
# Berechnet bisher nur die Winkel, filtert noch nichts.
# [INFO] [1758028728.591366353] [scan_info_node]: angle_min: -3.124, angle_max: 3.142, angle_increment: 0.008715
def restrict_scan_by_angle(scan, angle=None):
    if angle:
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_inc = scan.angle_increment
        zero_angle_forward_along_x = 0

        input_rad_angle = math.radians(angle)

        restricted_angle_min = zero_angle_forward_along_x - input_rad_angle/2
        restricted_angle_max = zero_angle_forward_along_x + input_rad_angle/2

        num_ranges = int(round((angle_max - angle_min) / angle_inc)) + 1

        angles_array = []
        current_angle = angle_min
        for _ in range(num_ranges):
            angles_array.append(current_angle)
            current_angle += angle_inc
