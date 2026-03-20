import RPi.GPIO as GPIO
import time
import sys
import threading
import cv2
import numpy as np
from picamera2 import Picamera2

# ==========================================
# HARDWARE PIN DEFINITIONS (BCM Numbering)
# ==========================================

# --- Motors ---
PWMA = 12  # Hardware PWM (Left)
AIN1 = 5
AIN2 = 6

PWMB = 16  # Software PWM (Right)
BIN1 = 20
BIN2 = 21

# --- Ultrasonic Sensors ---
# You didn't specify pins for these, so I selected free BCM pins. 
# PLEASE WIRE TO THESE OR CHANGE THEM TO MATCH YOUR WIRING!
# Reminder: Echo pins MUST use a voltage divider (resistors) before plugging into Pi!
US_FRONT_TRIG = 17 #right encoder
US_FRONT_ECHO = 27

US_LEFT_TRIG = 23 # left encoder
US_LEFT_ECHO = 22

US_RIGHT_TRIG = 24 #font 
US_RIGHT_ECHO = 25

# ==========================================
# GLOBAL VARIABLES
# ==========================================
pwm_a = None
pwm_b = None

# Distances in cm
dist_front = 999.0
dist_left = 999.0
dist_right = 999.0

# Base Motor Speed (0 to 100 duty cycle)
BASE_SPEED = 40
KP = 0.2  # Proportional steering constant

# ==========================================
# SETUP FUNCTIONS
# ==========================================
def setup_gpio():
    global pwm_a, pwm_b

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor Setup
    GPIO.setup([PWMA, PWMB, AIN1, AIN2, BIN1, BIN2], GPIO.OUT)
    pwm_a = GPIO.PWM(PWMA, 1000)
    pwm_b = GPIO.PWM(PWMB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)

    # Ultrasonic Setup
    GPIO.setup([US_FRONT_TRIG, US_LEFT_TRIG, US_RIGHT_TRIG], GPIO.OUT)
    GPIO.setup([US_FRONT_ECHO, US_LEFT_ECHO, US_RIGHT_ECHO], GPIO.IN)
    
    # Initialize Triggers to Low
    GPIO.output(US_FRONT_TRIG, False)
    GPIO.output(US_LEFT_TRIG, False)
    GPIO.output(US_RIGHT_TRIG, False)
    time.sleep(0.5)

def move_motors(speed_left, speed_right):
    # Clamp speeds to -100 to 100
    speed_left = max(-100, min(100, speed_left))
    speed_right = max(-100, min(100, speed_right))

    # --- LEFT MOTOR ---
    if speed_left > 0:
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(AIN2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(speed_left)
    elif speed_left < 0:
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.HIGH)
        pwm_a.ChangeDutyCycle(-speed_left)
    else:
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(0)

    # --- RIGHT MOTOR ---
    if speed_right > 0:
        GPIO.output(BIN1, GPIO.HIGH)
        GPIO.output(BIN2, GPIO.LOW)
        pwm_b.ChangeDutyCycle(speed_right)
    elif speed_right < 0:
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.HIGH)
        pwm_b.ChangeDutyCycle(-speed_right)
    else:
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.LOW)
        pwm_b.ChangeDutyCycle(0)

# ==========================================
# ULTRASONIC SENSOR LOGIC
# ==========================================
def read_ultrasonic(trig_pin, echo_pin):
    """Sends a ping and measures the time for the echo to return."""
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    start_time = time.time()
    stop_time = time.time()
    
    # Timeout logic so a disconnected sensor doesn't freeze the whole robot
    timeout = start_time + 0.04 

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
        if start_time > timeout:
            return 999.0  # Timeout

    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()
        if stop_time > timeout:
            return 999.0  # Timeout

    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2
    return distance

def sensor_thread_loop():
    """Continuously ranges the 3 sensors in the background."""
    global dist_front, dist_left, dist_right
    while True:
        dist_front = read_ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO)
        time.sleep(0.01) # Small delays prevent ultrasonic waves from bouncing into each other
        dist_left = read_ultrasonic(US_LEFT_TRIG, US_LEFT_ECHO)
        time.sleep(0.01)
        dist_right = read_ultrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO)
        time.sleep(0.01)

# ==========================================
# MAIN ROBOT LOOP
# ==========================================
def main():
    setup_gpio()

    # Start ultrasonic sensors in a background thread so they don't lag the camera
    sensor_thread = threading.Thread(target=sensor_thread_loop, daemon=True)
    sensor_thread.start()

    # Setup Camera
    print("Initializing Camera...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.start()
    
    print("Starting Robot Loop! Press 'q' in the video window to quit.")

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) # Fix PiCamera RGB color issue
            h, w = frame.shape[:2]
            frame_center_x = w // 2

            # Default motor states for this frame
            left_motor_speed = 0
            right_motor_speed = 0

            # ---------------------------------------------------------
            # 1. OBSTACLE AVOIDANCE OVERRIDE
            # ---------------------------------------------------------
            if dist_front < 15.0:
                # If something is very close in front, stop or reverse!
                cv2.putText(frame, "OBSTACLE FRONT! STOPPING", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                move_motors(0, 0)
            else:
                # ---------------------------------------------------------
                # 2. LINE TRACKING
                # ---------------------------------------------------------
                roi_start = h // 2
                roi = frame[roi_start:h, 0:w]

                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours) > 0:
                    c = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(c)

                    if area > 500:
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])

                            error = cx - frame_center_x

                            # Calculate Motor Speeds (Percentage 0-100)
                            left_motor_speed = BASE_SPEED + (KP * error)
                            right_motor_speed = BASE_SPEED - (KP * error)

                            # Visualizations for Line
                            x, y, bw, bh = cv2.boundingRect(c)
                            cv2.rectangle(frame, (x, y + roi_start), (x + bw, y + bh + roi_start), (0, 255, 0), 2)
                            cv2.circle(frame, (cx, cy + roi_start), 5, (255, 0, 0), -1)
                            cv2.line(frame, (frame_center_x, roi_start), (cx, cy + roi_start), (0, 0, 255), 2)
                
                # Apply the calculated speeds to the motors
                move_motors(left_motor_speed, right_motor_speed)

            # ---------------------------------------------------------
            # 3. BLUE CUBE DETECTION (Background tracking, doesn't control motors yet)
            # ---------------------------------------------------------
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([90, 100, 50])
            upper_blue = np.array([130, 255, 255])
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            blue_mask = cv2.GaussianBlur(blue_mask, (5, 5), 0)
            
            blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_blues = [cnt for cnt in blue_contours if cv2.contourArea(cnt) > 1000]
            
            if valid_blues:
                target_cube = max(valid_blues, key=cv2.contourArea)
                cx_b, cy_b, cw, ch = cv2.boundingRect(target_cube)
                cv2.rectangle(frame, (cx_b, cy_b), (cx_b + cw, cy_b + ch), (255, 0, 0), 3)
                cv2.putText(frame, "BLUE CUBE", (cx_b, cy_b - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            # ---------------------------------------------------------
            # 4. DRAW SENSOR HUD ON SCREEN
            # ---------------------------------------------------------
            cv2.putText(frame, f"US Front: {int(dist_front)} cm", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"US Left:  {int(dist_left)} cm", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"US Right: {int(dist_right)} cm", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            cv2.putText(frame, f"L_PWM: {int(left_motor_speed)} | R_PWM: {int(right_motor_speed)}", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Show Result
            cv2.imshow("Robot AI View", frame)

            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by User")

    finally:
        print("Cleaning up...")
        move_motors(0, 0)
        picam2.stop()
        cv2.destroyAllWindows()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
