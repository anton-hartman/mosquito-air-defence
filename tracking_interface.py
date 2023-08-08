from mosquito import Mosquito

''' This is just a guidline. Nothing enforces it.'''
class TrackingInterface:
    def __init__(self, max_age=5, min_hits=3):
        pass

    def track(self, centroids: list[tuple[int, int]]) -> list[Mosquito]: # type: ignore
        pass
