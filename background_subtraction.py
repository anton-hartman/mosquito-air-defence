import cv2

class BackgroundSubtractor:
    def __init__(self, first_frame, alpha=0.5):
        self.background = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY).astype(float)
        self.alpha = alpha

    def subtract(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(float)
        self.background = self.alpha * gray_frame + (1 - self.alpha) * self.background
        diff = cv2.absdiff(gray_frame, self.background)
        _, thresholded_diff = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
        return thresholded_diff
