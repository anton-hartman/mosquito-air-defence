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

# Example usage:
cap = cv2.VideoCapture('mosquito_data/many-mosquitoes-flying-white-bg.mp4')  # Open video file
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))  # Get total number of frames in the video
frame_counter = 0  # Variable to keep track of the current frame number

ret, frame = cap.read()  # Read the first frame

if ret:
    # Create a BackgroundSubtractor object with the first frame
    background_subtractor = BackgroundSubtractor(frame, alpha=0.01)

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

            # Resize frames for side-by-side display
            frame = cv2.resize(frame, (640, 480))
            subtracted_frame = cv2.resize(subtracted_frame, (640, 480))
            bg_frame = cv2.resize(background_subtractor.background, (640, 480))

            # Convert the images to 8-bit unsigned integer
            frame = cv2.convertScaleAbs(frame)
            subtracted_frame = cv2.convertScaleAbs(subtracted_frame)
            bg_frame = cv2.convertScaleAbs(bg_frame)

            # Create a side-by-side comparison
            comparison = np.hstack((frame, cv2.cvtColor(subtracted_frame, cv2.COLOR_GRAY2BGR), cv2.cvtColor(bg_frame, cv2.COLOR_GRAY2BGR)))

            # Display current frame number and total frames
            cv2.putText(comparison, f'Frame: {frame_counter}/{total_frames}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Detect blobs and draw circles
            blob_centroids = detect_blobs(subtracted_frame)
            for cx, cy in blob_centroids:
                cv2.circle(comparison, (cx, cy), 5, (0, 255, 0), 2)

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
