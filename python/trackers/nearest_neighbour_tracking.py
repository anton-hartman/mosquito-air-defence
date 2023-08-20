import numpy as np
from mosquito import Mosquito

class NearestNeighbourTracker:
    def __init__(self, max_distance=50, max_frames_missing=5):
        self.max_distance = max_distance  # Maximum distance to consider a centroid as a match
        self.max_frames_missing = max_frames_missing  # Maximum number of frames a centroid can be missing before considered lost
        self.next_mosquito_id = 0  # Counter to assign unique IDs to mosquitoes
        self.mosquitoes: list[Mosquito] = []  # List to store tracked mosquitoes

    def track(self, centroids):
        # Assign current frame centroids to existing mosquitoes
        for mosquito in self.mosquitoes:
            mosquito.matched = False  # Reset matched flag

        for centroid in centroids:
            best_match = None
            best_distance = self.max_distance

            for mosquito in self.mosquitoes:
                if not mosquito.matched:
                    distance = np.sqrt((centroid[0] - mosquito.centroid[0])**2 +
                                       (centroid[1] - mosquito.centroid[1])**2)

                    if distance < best_distance:
                        best_distance = distance
                        best_match = mosquito

            if best_match is not None:
                best_match.update(centroid)
            else:
                self.add_mosquito(centroid)

        # Remove lost mosquitoes
        self.mosquitoes = [mosquito for mosquito in self.mosquitoes if mosquito.frames_missing <= self.max_frames_missing]

        # Update frames missing for unmatched mosquitoes
        for mosquito in self.mosquitoes:
            if not mosquito.matched:
                mosquito.frames_missing += 1

        return self.mosquitoes

    def add_mosquito(self, centroid):
        mosquito = Mosquito(self.next_mosquito_id, centroid)
        self.next_mosquito_id += 1
        self.mosquitoes.append(mosquito)
