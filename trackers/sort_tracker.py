import numpy as np
from numpy import ndarray
# from typing import Union
# from typing import Any
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment

from mosquito import Mosquito

class SORT:
    def __init__(self, max_age=5, min_hits=1):
        self.max_age = max_age
        self.min_hits = min_hits
        self.trackers: list[Tracker] = []
        self.frame_count = 0

    def update(self, detections: list[tuple[int, int, int, int]]) -> list[Mosquito]:
        '''Params: detections: list of bounding boxes in the format (cx, cy, w, h)'''
        self.frame_count += 1
        # Update trackers
        for tracker in self.trackers:
            tracker.predict()

        # Match detections to trackers
        if len(detections) > 0:
            # Create cost matrix
            cost_matrix: ndarray = np.zeros((len(self.trackers), len(detections)))
            for i, tracker in enumerate(self.trackers):
                for j, detection in enumerate(detections):
                    cost_matrix[i, j] = tracker.distance(detection)

            # Solve assignment problem
            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            # Update trackers with assigned detections
            for i, j in zip(row_ind, col_ind):
                self.trackers[i].update(detections[j])
                self.trackers[i].hits += 1

            # Create new trackers for unassigned detections
            unassigned_detections: ndarray = np.delete(detections, col_ind, axis=0)
            for detection in unassigned_detections:
                tracker = Tracker()
                tracker.update(detection)
                self.trackers.append(tracker)

        # Remove trackers that have been lost for too long
        self.trackers: list[Tracker] = [tracker for tracker in self.trackers if tracker.age < self.max_age and tracker.hits >= self.min_hits]

        # Return list of tracked objects
        objects: list[list[float]] = []
        mos: list[Mosquito] = []
        for tracker in self.trackers:
            if tracker.age < self.max_age and tracker.hits >= self.min_hits:
                objects.append(tracker.get_state())
                mos.append(tracker.get_mosquito())
        # return objects
        return mos

class Tracker:
    def __init__(self):
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.array([[1, 0, 1, 0],
                              [0, 1, 0, 1],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]])
        self.kf.P *= 1000
        self.kf.R = np.array([[0.1, .0],
                              [.0, 0.1]])
        self.kf.Q = np.array([[1, 0, 1, 0],
                              [0, 1, 0, 1],
                              [1, 0, 1, 0],
                              [0, 1, 0, 1]]) * 0.01
        self.hits = 0
        self.age = 0
        self.x = np.zeros((4, 1))
        self.P = self.kf.P
        self.id = Tracker.next_id()
        Tracker.increment_next_id()

    def predict(self):
        self.x = self.kf.predict()
        self.P = self.kf.P
        self.age += 1

    def update(self, z):
        self.kf.update(z)
        self.hits += 1
        self.age = 0

    def distance(self, z) -> float:
        return np.sqrt((self.x[0] - z[0])**2 + (self.x[1] - z[1])**2)

    def get_state(self) -> list[float]:
        return [self.x[0], self.x[1], self.x[2], self.x[3], self.id]
    
    def get_mosquito(self) -> Mosquito:
        return Mosquito(self.id, (int(self.x[0][0]), int(self.x[1][0])))

    @staticmethod
    def next_id():
        if not hasattr(Tracker, 'id'):
            Tracker.id = 0
        return Tracker.id

    @staticmethod
    def increment_next_id():
        Tracker.id += 1