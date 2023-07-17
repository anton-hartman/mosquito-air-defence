class Mosquito:
    def __init__(self, mosquito_id, centroid):
        self.id = mosquito_id
        self.centroid = centroid
        self.frames_missing = 0
        self.matched = True

    def update(self, centroid):
        self.centroid = centroid
        self.frames_missing = 0
        self.matched = True