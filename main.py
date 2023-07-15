import math
import cv2
import numpy as np
from background_subtraction import BackgroundSubtractor
from blob_detection import detect_blobs
# from nearest_neighbour_tracking import MosquitoTracker
from kalman_filter import KF as KalmanFilter

class MosquitoTrackingInterface:
    def __init__(self, frame_resize_factor = 1.0, darkmode=False, comparison=False):
        self.frame_resize_factor = frame_resize_factor
        self.darkmode = darkmode
        self.comparison = comparison

    def start_tracking(self, video_path):
        # Open the video file
        cap = cv2.VideoCapture(video_path)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        total_frames = 103
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        print('FPS:', fps)
        
        # Initialize the frame index and step
        frame_index = 0
        step = 1

        # Read the first frame
        ret, frame = cap.read()
        if not ret:
            print('Failed to read the video file.')
            return
        frame_size = tuple(map(lambda x: int(x * self.frame_resize_factor), frame.shape[1::-1]))
        frame = cv2.resize(frame, frame_size)

        # Initialize the background subtractor
        background_subtractor = BackgroundSubtractor(frame, alpha=0.000001)

        # Initialize the mosquito tracker
        # tracker = MosquitoTracker()

        # Variables for video control
        is_paused = False
        delay = 100   # Default delay (in milliseconds) between frames

        init = False
        while True:
            if not is_paused:
                ret, frame = cap.read()
                if not ret:
                    print('End of video file.')
                    break
                frame = cv2.resize(frame, frame_size)

                # Perform background subtraction
                subtracted_frame = background_subtractor.subtract(frame)

                # Detect blobs and get centroid coordinates
                blob_centroids = detect_blobs(subtracted_frame)
                # if not blob_centroids:
                #     continue

                # Track mosquitoes based on centroids
                # tracked_mosquitoes = tracker.track(blob_centroids)

                if not init and blob_centroids:
                    kf = KalmanFilter(initial_x=blob_centroids[0][0],
                                      initial_vx=0,
                                      x_accel_variance=300,
                                      initial_y=blob_centroids[0][1],
                                      initial_vy=0,
                                      y_accel_variance=300) 
                    init = True
                elif init:  
                    if blob_centroids:
                        updated = kf.update(blob_centroids[0][0], blob_centroids[0][1])
                        cv2.circle(frame, (blob_centroids[0][0], blob_centroids[0][1]), 10, (0, 255, 0), 2)
                        cv2.circle(frame, updated, 10, (255, 0, 0), 2)

                    predicted = kf.predict(1/fps)
                    cv2.circle(frame, predicted, 10, (0 , 0, 255), 2)
                
                # # Draw circles around tracked mosquitoes
                # for mosquito in tracked_mosquitoes:
                #     cx, cy = mosquito.last_centroid
                #     if mosquito.is_matched:
                #         color = (0, 255, 0)  # Green color for matched mosquitoes
                #     else:
                #         color = (0, 0, 255)  # Red color for unmatched mosquitoes
                #     cv2.circle(frame, (cx, cy), 5, color, 2)
                #     cv2.putText(frame, str(mosquito.mosquito_id), (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                #     cv2.circle(frame, predicted, 5, (255 , 0, 0), 2)

                if self.comparison:
                    # Convert the images to 8-bit unsigned integer
                    # frame = cv2.convertScaleAbs(frame)
                    subtracted_frame = cv2.convertScaleAbs(subtracted_frame)
                    bg_frame = cv2.convertScaleAbs(background_subtractor.background)

                    # Create a side-by-side comparison
                    comparison = np.hstack((frame, cv2.cvtColor(subtracted_frame, cv2.COLOR_GRAY2BGR), cv2.cvtColor(bg_frame, cv2.COLOR_GRAY2BGR)))

                    # Display current frame number and total frames
                    cv2.putText(comparison, f'Frame: {frame_index}/{total_frames}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

                    cv2.imshow('Original vs. Subtracted vs. Background', comparison)
                else:
                    # Display current frame number and total frames
                    cv2.putText(frame, f'Frame: {frame_index}/{total_frames}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

                    if self.darkmode:
                        # Adjust the brightness and contrast of the image
                        alpha = 0.6  # Contrast control (1.0-3.0)
                        beta = 60  # Brightness control (0-100)
                        frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
                    
                    cv2.imshow('Tracking', frame)

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

            # Update the frame index
            frame_index += step

            # Check if the frame index is out of bounds
            if frame_index < 0 or frame_index >= total_frames - 1:
                # If so, set the frame index to the first or last frame and reverse the step
                frame_index = 0 if step < 0 else total_frames - 1
                step = -step

            # Set the current frame position
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)

        cap.release()
        cv2.destroyAllWindows()


mti = MosquitoTrackingInterface(frame_resize_factor=1, comparison=True)
# mti.start_tracking('mosquito_data/many-mosquitoes-flying-white-bg.mp4')
mti.start_tracking('mosquito_data/black-dot-bouncing-across.mp4')
# mti.start_tracking('mosquito_data/squash-ball-rolling.mp4')