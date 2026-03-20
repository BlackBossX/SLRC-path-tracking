from picamera2 import Picamera2
import cv2
import numpy as np


picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Basic parameters for motor simulation
BASE_RPM = 100
KP = 0.5  # Proportional control constant

while True:
    # capture_array() returns RGB by default on PiCamera2
    frame = picam2.capture_array()
    
    # ---------------------------------------------------------
    # FIX COLOR ISSUE: Convert RGB to OpenCV's native BGR format. 
    # This solves the issue where blue looks yellow/orange!
    # ---------------------------------------------------------
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
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

    # ---------------------------------------------------------
    # BLUE CUBE DETECTION
    # ---------------------------------------------------------
    # Convert whole frame to HSV to isolate the color blue
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define HSV color boundaries for Blue
    lower_blue = np.array([90, 100, 50])
    upper_blue = np.array([130, 255, 255])
    
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_mask = cv2.GaussianBlur(blue_mask, (5, 5), 0)
    
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter out small noise spots
    valid_blues = [cnt for cnt in blue_contours if cv2.contourArea(cnt) > 1000]
    
    if valid_blues:
        # Assume the largest blue object is our target cube
        target_cube = max(valid_blues, key=cv2.contourArea)
        cx_b, cy_b, cw, ch = cv2.boundingRect(target_cube)
        
        # Keep track of it by drawing a frame around it (Color format BGR -> 255, 0, 0 is Blue)
        cv2.rectangle(frame, (cx_b, cy_b), (cx_b + cw, cy_b + ch), (255, 0, 0), 3)
        cv2.putText(frame, "BLUE CUBE TRACKED", (cx_b, cy_b - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    # Show the tracking result
    cv2.imshow("Robot Vision", frame)

    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()