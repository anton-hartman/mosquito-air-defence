import cv2

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