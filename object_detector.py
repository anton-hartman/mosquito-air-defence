import cv2
import numpy as np

class ObjectDetector:
    class BackgroundSubtractor:
        def __init__(self, first_frame: np.ndarray, alpha: float) -> None:
            self.background = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY).astype(float)
            self.alpha = alpha
            self.subtracted_frame = self.background.copy() # Just a placeholder

        def subtract(self, frame) -> np.ndarray:
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(float)
            self.background = self.alpha * gray_frame + (1 - self.alpha) * self.background
            diff = cv2.absdiff(gray_frame, self.background)
            _, thresholded_diff = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
            return thresholded_diff
        
    def __init__(self, first_frame, alpha=0.01) -> None:
        self.bg_subtractor = self.BackgroundSubtractor(first_frame, alpha=alpha)

    def detect_blobs(self, binarized_image) -> list[tuple[int, int]]:
        # Convert the input image to CV_8UC1 format
        binarized_image = cv2.convertScaleAbs(binarized_image)

        # Find contours in the binarized image
        contours, _ = cv2.findContours(binarized_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on area
        min_area = 1  # Minimum contour area to consider
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

        # Compute centroid for each filtered contour
        blob_centroids = []
        for cnt in filtered_contours:
            moments = cv2.moments(cnt)
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            blob_centroids.append((cx, cy))

        return blob_centroids
    
    def detect_objects(self, frame) -> list[tuple[int, int]]:
        # Perform background subtraction
        self.subtracted_frame = self.bg_subtractor.subtract(frame)

        # Detect blobs and get centroid coordinates
        blob_centroids = self.detect_blobs(binarized_image=self.subtracted_frame)

        return blob_centroids

    
    @property
    def background(self) -> np.ndarray:
        return self.bg_subtractor.background
    
    @property
    def subtracted_frame(self) -> np.ndarray:
        return self.bg_subtractor.subtracted_frame
    
    @subtracted_frame.setter
    def subtracted_frame(self, frame) -> None:
        self.bg_subtractor.subtracted_frame = frame