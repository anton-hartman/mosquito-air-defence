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

    def __str__(self):
        return f'Mosquito {self.id} at {self.centroid}'
    
    def __repr__(self):
        return str(self)