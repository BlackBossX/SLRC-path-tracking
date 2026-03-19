from picamera2 import Picamera2
import cv2
import numpy as np


def identify_shape(contour):
    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
    vertices = len(approx)

    if vertices == 3:
        return "Triangle"
    if vertices == 4:
        x, y, w, h = cv2.boundingRect(approx)
        if h == 0:
            return "Rectangle"
        aspect_ratio = w / float(h)
        if 0.95 <= aspect_ratio <= 1.05:
            return "Square"
        return "Rectangle"
    if vertices > 4:
        return "Circle"
    return "Object"


picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Basic parameters for motor simulation
BASE_RPM = 100
KP = 0.5  # Proportional control constant

while True:
    frame = picam2.capture_array()
    h, w = frame.shape[:2]
    frame_center_x = w // 2

    # Focus on the bottom half of the frame where the ground/line usually is
    roi_start = h // 2
    roi = frame[roi_start:h, 0:w]

    # Convert ROI to grayscale and blur it
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Threshold for a BLACK line: pixel values < 60 become white (255)
    _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Assume the largest contour is the track/line
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > 500: # Filter out small noise
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Calculate Line Error
                # If error > 0, line is to the right. If error < 0, line is to the left.
                error = cx - frame_center_x

                # Motor Speed Calculation (Differential Drive Steering)
                # If line is to the right (error > 0), steer right: increase Left RPM, decrease Right RPM
                left_rpm = BASE_RPM + (KP * error)
                right_rpm = BASE_RPM - (KP * error)

                # Clamp values to valid motor PWM/RPM ranges (e.g. 0 to 200)
                left_rpm = max(0, min(200, left_rpm))
                right_rpm = max(0, min(200, right_rpm))

                # Visualizations
                # 1. Bounding Box around the line
                x, y, bw, bh = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y + roi_start), (x + bw, y + bh + roi_start), (0, 255, 0), 2)

                # 2. Draw centroid & centerline to show error
                cv2.circle(frame, (cx, cy + roi_start), 5, (255, 0, 0), -1)
                cv2.line(frame, (frame_center_x, roi_start), (cx, cy + roi_start), (0, 0, 255), 2)

                # 3. Text output on screen
                cv2.putText(frame, f"Error: {error} px", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, f"Left RPM: {int(left_rpm)}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Right RPM: {int(right_rpm)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        # If no line is found
        cv2.putText(frame, "No Line Detected", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Line Tracker", frame)

    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()