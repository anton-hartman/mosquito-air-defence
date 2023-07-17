import cv2
from mosquito import Mosquito

# KCF (Kernelized Correlation Filter)
class KcfTracker:
    def __init__(self):
        self.trackers = {}
        self.mosquitoes = {}

    def track(self, frame, centroids: list[tuple[int, int]]) -> list[Mosquito]:
        # Initialize new trackers for new centroids
        for i, centroid in enumerate(centroids):
            if i not in self.trackers:
                tracker = cv2.TrackerKCF_create()
                tracker.init(frame, (centroid[0], centroid[1], 20, 20))
                self.trackers[i] = tracker
                mosquito = Mosquito(i, centroid)
                self.mosquitoes[i] = mosquito

        # Update existing trackers
        for i in list(self.trackers.keys()):
            success, bbox = self.trackers[i].update(frame)

            if success:
                x, y, w, h = [int(v) for v in bbox]
                centroid = (x + w//2, y + h//2)
                self.mosquitoes[i].update(centroid)
            else:
                del self.trackers[i]
                del self.mosquitoes[i]

        return list(self.mosquitoes.values())