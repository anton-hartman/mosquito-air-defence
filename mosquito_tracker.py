import cv2
import numpy as np

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

def detect_blobs(binarized_image):
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

class MosquitoTracker:
    def __init__(self, max_distance=50, max_frames_missing=5):
        self.max_distance = max_distance  # Maximum distance to consider a centroid as a match
        self.max_frames_missing = max_frames_missing  # Maximum number of frames a centroid can be missing before considered lost
        self.next_mosquito_id = 0  # Counter to assign unique IDs to mosquitoes
        self.mosquitoes = []  # List to store tracked mosquitoes

    def track(self, centroids):
        # Assign current frame centroids to existing mosquitoes
        for mosquito in self.mosquitoes:
            mosquito.is_matched = False  # Reset matched flag

        for centroid in centroids:
            best_match = None
            best_distance = self.max_distance

            for mosquito in self.mosquitoes:
                if not mosquito.is_matched:
                    distance = np.sqrt((centroid[0] - mosquito.last_centroid[0])**2 +
                                       (centroid[1] - mosquito.last_centroid[1])**2)

                    if distance < best_distance:
                        best_distance = distance
                        best_match = mosquito

            if best_match is not None:
                best_match.update(centroid)
                best_match.is_matched = True
            else:
                self.add_mosquito(centroid)

        # Remove lost mosquitoes
        self.mosquitoes = [mosquito for mosquito in self.mosquitoes if mosquito.frames_missing <= self.max_frames_missing]

        # Update frames missing for unmatched mosquitoes
        for mosquito in self.mosquitoes:
            if not mosquito.is_matched:
                mosquito.frames_missing += 1

    def add_mosquito(self, centroid):
        mosquito = Mosquito(self.next_mosquito_id, centroid)
        self.next_mosquito_id += 1
        self.mosquitoes.append(mosquito)

    def get_tracked_mosquitoes(self):
        return self.mosquitoes


class Mosquito:
    def __init__(self, mosquito_id, centroid):
        self.mosquito_id = mosquito_id
        self.last_centroid = centroid
        self.frames_missing = 0
        self.is_matched = True

    def update(self, centroid):
        self.last_centroid = centroid
        self.frames_missing = 0


# Example usage:
cap = cv2.VideoCapture('mosquito_data/many-mosquitoes-flying-white-bg.mp4')  # Open video file
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))  # Get total number of frames in the video
frame_counter = 0  # Variable to keep track of the current frame number

ret, frame = cap.read()  # Read the first frame

if ret:
    # Create a BackgroundSubtractor object with the first frame
    background_subtractor = BackgroundSubtractor(frame, alpha=0.01)

    # Create a MosquitoTracker object
    tracker = MosquitoTracker()

    # Variables for video control
    is_paused = False
    delay = 1  # Default delay (in milliseconds) between frames

    while True:
        if not is_paused:
            ret, frame = cap.read()  # Read a new frame
            if not ret:
                break
            frame_counter += 1  # Increment the frame counter

            # Perform background subtraction
            subtracted_frame = background_subtractor.subtract(frame)

            # Detect blobs and get centroid coordinates
            blob_centroids = detect_blobs(subtracted_frame)

            # Track mosquitoes based on centroids
            tracker.track(blob_centroids)

            # Get tracked mosquitoes
            tracked_mosquitoes = tracker.get_tracked_mosquitoes()

            # Draw circles around tracked mosquitoes
            for mosquito in tracked_mosquitoes:
                cx, cy = mosquito.last_centroid
                if mosquito.is_matched:
                    color = (0, 255, 0)  # Green color for matched mosquitoes
                else:
                    color = (0, 0, 255)  # Red color for unmatched mosquitoes
                cv2.circle(frame, (cx, cy), 5, color, 2)
                cv2.putText(frame, str(mosquito.mosquito_id), (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # # Resize frames for side-by-side display
            # frame = cv2.resize(frame, (640, 480))
            # subtracted_frame = cv2.resize(subtracted_frame, (640, 480))
            # bg_frame = cv2.resize(background_subtractor.background, (640, 480))
            bg_frame = background_subtractor.background

            # Convert the images to 8-bit unsigned integer
            frame = cv2.convertScaleAbs(frame)
            subtracted_frame = cv2.convertScaleAbs(subtracted_frame)
            bg_frame = cv2.convertScaleAbs(bg_frame)

            # Create a side-by-side comparison
            comparison = np.hstack((frame, cv2.cvtColor(subtracted_frame, cv2.COLOR_GRAY2BGR), cv2.cvtColor(bg_frame, cv2.COLOR_GRAY2BGR)))

            # Display current frame number and total frames
            cv2.putText(comparison, f'Frame: {frame_counter}/{total_frames}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('Original vs. Subtracted', comparison)

        speeds = {ord('['): 100, ord(']'): -100}  # Mapping of key presses to speed factors
        key = cv2.waitKey(delay) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):  # Spacebar to pause/resume
            is_paused = not is_paused
        elif key in speeds:  # Change speed
            delay += speeds[key]
        elif key == ord('r'):
            delay = 1

    cap.release()
    cv2.destroyAllWindows()
else:
    print('Failed to read the video file.')
