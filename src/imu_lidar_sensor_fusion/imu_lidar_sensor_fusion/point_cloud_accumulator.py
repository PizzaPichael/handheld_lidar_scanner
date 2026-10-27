class PointCloudAccumulator:
    """Sammelt die Punkte aller Scans zu einer Gesamtpunktwolke.

    Aktuell wird jeder Punkt gespeichert, die Wolke wächst also unbegrenzt.
    """

    def __init__(self):
        self.points = []

    def add(self, points):
        self.points.extend(points)
