import cv2
import numpy as np

class BackgroundSubtractor:
    def __init__(self, first_frame):
        self.background = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)

    def subtract(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(gray_frame, self.background)
        _, thresholded = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
        return thresholded

# Example usage:
cap = cv2.VideoCapture('mosquito_data/many-mosquitoes-flying-white-bg.mp4')  # Open video file
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))  # Get total number of frames in the video
frame_counter = 0  # Variable to keep track of the current frame number

ret, frame = cap.read()  # Read the first frame

if ret:
    # Create a BackgroundSubtractor object with the first frame
    background_subtractor = BackgroundSubtractor(frame)

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

            # Create a side-by-side comparison
            comparison = np.hstack((frame, cv2.cvtColor(subtracted_frame, cv2.COLOR_GRAY2BGR)))

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