import math


class PointCloudAccumulator:
    """Sammelt die Punkte aller Scans zu einer Gesamtpunktwolke (Voxel Grid Downsampling).

    Der Raum ist in Würfel mit Kantenlänge voxel_size (m) eingeteilt. Pro Würfel wird nur der
    erste Punkt gespeichert, alle weiteren im selben Würfel werden verworfen. Die Wolke wächst
    dadurch nur, wenn neue Bereiche gescannt werden, nicht mit der Scandauer.
    """

    def __init__(self, voxel_size):
        self.voxel_size = voxel_size
        self._voxels = {}  # (ix, iy, iz) -> (x, y, z)

    def add(self, points):
        s = self.voxel_size
        for p in points:
            key = (math.floor(p[0] / s), math.floor(p[1] / s), math.floor(p[2] / s))
            if key not in self._voxels:
                self._voxels[key] = p

    @property
    def points(self):
        return list(self._voxels.values())
