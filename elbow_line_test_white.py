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

# --- Encoders ---
ENC_LEFT_A = 24
ENC_LEFT_B = 25

ENC_RIGHT_A = 17
ENC_RIGHT_B = 27

# --- Ultrasonic Sensors ---
US_RIGHT_TRIG = 9
US_RIGHT_ECHO = 10

US_LEFT_TRIG = 23
US_LEFT_ECHO = 22

US_FRONT_TRIG = 8
US_FRONT_ECHO = 7

# ==========================================
# GLOBAL VARIABLES
# ==========================================
pwm_a = None
pwm_b = None

# Encoder Counts
left_encoder_count = 0
right_encoder_count = 0

# Distances in cm
dist_front = 999.0
dist_left = 999.0
dist_right = 999.0

# Base Motor Speed
BASE_SPEED = 20 
KP = 0.002  
KI = 0.0  
KD = 0.0005 

last_error = 0
integral = 0

# ==========================================
# ENDODER ISR FUNCTIONS
# ==========================================
def left_encoder_isr(channel):
    global left_encoder_count
    if GPIO.input(ENC_LEFT_B) == GPIO.HIGH:
        left_encoder_count += 1
    else:
        left_encoder_count -= 1

def right_encoder_isr(channel):
    global right_encoder_count
    if GPIO.input(ENC_RIGHT_B) == GPIO.HIGH:
        right_encoder_count += 1
    else:
        right_encoder_count -= 1

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

    # Encoder Setup
    GPIO.setup([ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B], GPIO.IN)
    GPIO.add_event_detect(ENC_LEFT_A, GPIO.RISING, callback=left_encoder_isr, bouncetime=5)
    GPIO.add_event_detect(ENC_RIGHT_A, GPIO.RISING, callback=right_encoder_isr, bouncetime=5)

    # Ultrasonic Setup
    GPIO.setup([US_FRONT_TRIG, US_LEFT_TRIG, US_RIGHT_TRIG], GPIO.OUT)
    GPIO.setup([US_FRONT_ECHO, US_LEFT_ECHO, US_RIGHT_ECHO], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    GPIO.output(US_FRONT_TRIG, GPIO.LOW)
    GPIO.output(US_LEFT_TRIG, GPIO.LOW)
    GPIO.output(US_RIGHT_TRIG, GPIO.LOW)
    time.sleep(1)

def move_motors(speed_left, speed_right):
    speed_left = max(-100, min(100, speed_left))
    speed_right = max(-100, min(100, speed_right))

    # --- LEFT MOTOR ---
    if speed_left > 0:
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.HIGH)
        pwm_a.ChangeDutyCycle(speed_left)
    elif speed_left < 0:
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(AIN2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(-speed_left)
    else:
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(0)

    # --- RIGHT MOTOR ---
    if speed_right > 0:
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.HIGH)
        pwm_b.ChangeDutyCycle(speed_right)
    elif speed_right < 0:
        GPIO.output(BIN1, GPIO.HIGH)
        GPIO.output(BIN2, GPIO.LOW)
        pwm_b.ChangeDutyCycle(-speed_right)
    else:
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.LOW)
        pwm_b.ChangeDutyCycle(0)

# ==========================================
# ULTRASONIC SENSOR LOGIC
# ==========================================
def read_ultrasonic(trig_pin, echo_pin):
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    pulse_send = time.time()
    pulse_received = time.time()
    timeout = time.time() + 0.05 

    while GPIO.input(echo_pin) == 0:
        pulse_send = time.time()
        if time.time() > timeout:
            return 999.0

    while GPIO.input(echo_pin) == 1:
        pulse_received = time.time()
        if time.time() > timeout:
            return 999.0

    pulse_duration = pulse_received - pulse_send
    
    if pulse_duration > 0 and pulse_duration < 0.05:
        return 34300 * (pulse_duration / 2.0)
    return 999.0

def sensor_thread_loop():
    global dist_front, dist_left, dist_right
    while True:
        dist_front = read_ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO)
        time.sleep(0.02)
        dist_left = read_ultrasonic(US_LEFT_TRIG, US_LEFT_ECHO)
        time.sleep(0.02)
        dist_right = read_ultrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO)
        time.sleep(0.02)

# ==========================================
# MAIN ROBOT LOOP
# ==========================================
def main():
    setup_gpio()

    sensor_thread = threading.Thread(target=sensor_thread_loop, daemon=True)
    sensor_thread.start()

    print("Initializing Camera & Elbow Integration Logic...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.set_controls({"ExposureValue": 0})
    picam2.start()
    
    print("Starting Robot! Tracking WHITE line instead of BLACK. Press 'q' to quit.")

    # We use last_known_error to remember the line position if it temporarily leaves the frame
    global last_error, integral
    last_known_error = 0

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]
            frame_center_x = w // 2

            left_motor_speed = 0
            right_motor_speed = 0

            if dist_front < 15.0:
                cv2.putText(frame, "OBSTACLE FRONT! STOPPING", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                move_motors(0, 0)
            else:
                # Focus on the bottom half of the frame
                roi_start = h // 2
                roi = frame[roi_start:h, 0:w]

                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                
                # CHANGED FOR WHITE LINE TRACKING: Use THRESH_BINARY and a threshold for bright pixels
                # Values > 180 become white (255), everything else black (0)
                _, thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
                
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                line_detected = False

                if len(contours) > 0:
                    c = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(c)

                    if area > 500:
                        line_detected = True
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])

                            error = cx - frame_center_x
                            last_known_error = error  # Store this in case we lose the line

                            integral += error
                            derivative = error - last_error
                            last_error = error

                            # ==========================================
                            # ELBOW / ALIGNMENT LOGIC
                            # ==========================================
                            # If the line is spotted way over on the left (the elbow), gradually pivot left
                            if error < -100:
                                cv2.putText(frame, "ALIGNING LEFT (ELBOW)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                                left_motor_speed = -10
                                right_motor_speed = BASE_SPEED + 10

                            # If the line is spotted way over on the right, gradually pivot right
                            elif error > 100:
                                cv2.putText(frame, "ALIGNING RIGHT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                                left_motor_speed = BASE_SPEED + 10
                                right_motor_speed = -10

                            # Normal Tracking
                            else:
                                cv2.putText(frame, "TRACKING WHITE LINE", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                turn_adjustment = (KP * error) + (KI * integral) + (KD * derivative)
                                left_motor_speed = BASE_SPEED + turn_adjustment
                                right_motor_speed = BASE_SPEED - turn_adjustment

                            # Drawing Visuals
                            x, y, bw, bh = cv2.boundingRect(c)
                            cv2.rectangle(frame, (x, y + roi_start), (x + bw, y + bh + roi_start), (0, 255, 0), 2)
                            cv2.circle(frame, (cx, cy + roi_start), 5, (255, 0, 0), -1)
                            cv2.line(frame, (frame_center_x, roi_start), (cx, cy + roi_start), (0, 0, 255), 2)

                # ==========================================
                # NO LINE / SEEKING LOGIC
                # ==========================================
                if not line_detected:
                    # We lost the line while it was far left - keep pivoting left to find it again
                    if last_known_error < -100:
                        cv2.putText(frame, "LOST - SEEKING LEFT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        left_motor_speed = -10
                        right_motor_speed = BASE_SPEED + 10

                    # We lost the line while it was far right - keep pivoting right
                    elif last_known_error > 100:
                        cv2.putText(frame, "LOST - SEEKING RIGHT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        left_motor_speed = BASE_SPEED + 10
                        right_motor_speed = -10
                        
                    # Initial state (last_known_error == 0): We haven't seen the line yet. Move forward to find the track.
                    else:
                        cv2.putText(frame, "NO LINE - SEEKING FORWARD", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        left_motor_speed = BASE_SPEED
                        right_motor_speed = BASE_SPEED

                move_motors(left_motor_speed, right_motor_speed)

            # Draw HUD
            cv2.putText(frame, f"US Front: {int(dist_front)} cm", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"L_PWM: {int(left_motor_speed)} | R_PWM: {int(right_motor_speed)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
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