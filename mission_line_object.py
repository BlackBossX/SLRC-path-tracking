import RPi.GPIO as GPIO
import time
import sys
import serial
import threading
import cv2
import numpy as np
from picamera2 import Picamera2

# ==========================================
# HARDWARE PIN DEFINITIONS (BCM Numbering)
# ==========================================
# --- Motors ---
PWMA = 12
AIN1 = 5
AIN2 = 6

PWMB = 16
BIN1 = 20
BIN2 = 21

# --- Ultrasonic Sensors ---
US_FRONT_TRIG = 8
US_FRONT_ECHO = 7

US_LEFT_TRIG = 23
US_LEFT_ECHO = 22

US_RIGHT_TRIG = 9
US_RIGHT_ECHO = 10

# ==========================================
# CALIBRATION CONSTANTS
# ==========================================
# Using the values from your object_detect2.py. 
# Make sure to update TICKS_PER_DEGREE_HOT with your true calibrated value!
TICKS_PER_DEGREE_HOT = 0.58  

# Line Tracking PID Constants
BASE_SPEED = 25
KP = 0.002  
KI = 0.0  
KD = 0.0005 

last_error = 0
integral = 0

# ==========================================
# GLOBAL VARIABLES
# ==========================================
raw_left_encoder     = 0
raw_right_encoder    = 0
left_encoder_offset  = 0
right_encoder_offset = 0
left_encoder_count   = 0
right_encoder_count  = 0

dist_front = 999.0
dist_left  = 999.0
dist_right = 999.0

pwm_a   = None
pwm_b   = None
running = True


# ==========================================
# BACKGROUND THREADS
# ==========================================
def serial_read_thread():
    global raw_left_encoder, raw_right_encoder
    global left_encoder_count, right_encoder_count
    global running

    ports_to_try = ['/dev/serial0', '/dev/ttyS0', '/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyACM0']
    arduino = None

    for port in ports_to_try:
        try:
            arduino = serial.Serial(port, 115200, timeout=1)
            print(f"Connected to Arduino Encoders on {port}")
            break
        except Exception:
            continue

    if arduino is None:
        print("\nERROR: Could not find Arduino on any port.")
        running = False
        sys.exit(1)

    while running:
        try:
            if arduino.in_waiting > 0:
                line = arduino.readline().decode('utf-8', errors='ignore').strip()
                if ',' in line:
                    parts = line.split(',')
                    if len(parts) == 2:
                        try:
                            raw_left_encoder  = int(parts[0])
                            raw_right_encoder = int(parts[1])
                            left_encoder_count  = raw_left_encoder  - left_encoder_offset
                            right_encoder_count = raw_right_encoder - right_encoder_offset
                        except ValueError:
                            pass
            time.sleep(0.005)
        except Exception:
            pass

    if arduino and arduino.is_open:
        arduino.close()


def read_ultrasonic(trig_pin, echo_pin):
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    pulse_send     = time.time()
    pulse_received = time.time()
    timeout        = time.time() + 0.05

    while GPIO.input(echo_pin) == 0:
        pulse_send = time.time()
        if time.time() > timeout:
            return 999.0

    while GPIO.input(echo_pin) == 1:
        pulse_received = time.time()
        if time.time() > timeout:
            return 999.0

    pulse_duration = pulse_received - pulse_send
    if 0 < pulse_duration < 0.05:
        return 34300 * (pulse_duration / 2.0)
    return 999.0


def sensor_thread_loop():
    global dist_front, dist_left, dist_right, running
    while running:
        dist_front = read_ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO)
        time.sleep(0.02)
        if not running:
            break
        dist_left = read_ultrasonic(US_LEFT_TRIG, US_LEFT_ECHO)
        time.sleep(0.02)
        if not running:
            break
        dist_right = read_ultrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO)
        time.sleep(0.02)


# ==========================================
# SETUP AND MOTOR LOGIC
# ==========================================
def setup_gpio():
    global pwm_a, pwm_b

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup([PWMA, PWMB, AIN1, AIN2, BIN1, BIN2], GPIO.OUT)
    GPIO.setup([US_FRONT_TRIG, US_LEFT_TRIG, US_RIGHT_TRIG], GPIO.OUT)
    GPIO.setup([US_FRONT_ECHO, US_LEFT_ECHO, US_RIGHT_ECHO],
               GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.output(US_FRONT_TRIG, GPIO.LOW)
    GPIO.output(US_LEFT_TRIG,  GPIO.LOW)
    GPIO.output(US_RIGHT_TRIG, GPIO.LOW)

    pwm_a = GPIO.PWM(PWMA, 1000)
    pwm_b = GPIO.PWM(PWMB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)

    threading.Thread(target=serial_read_thread, daemon=True).start()
    threading.Thread(target=sensor_thread_loop, daemon=True).start()

    time.sleep(1)


def move_motors(speed_left, speed_right):
    speed_left  = max(-100, min(100, speed_left))
    speed_right = max(-100, min(100, speed_right))

    # NOTE: These AIN/BIN directions match the accurate "drive_until_object" logic that was working
    if speed_left > 0:
        GPIO.output(AIN1, GPIO.LOW);  GPIO.output(AIN2, GPIO.HIGH)
        pwm_a.ChangeDutyCycle(speed_left)
    elif speed_left < 0:
        GPIO.output(AIN1, GPIO.HIGH); GPIO.output(AIN2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(-speed_left)
    else:
        GPIO.output(AIN1, GPIO.LOW);  GPIO.output(AIN2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(0)

    if speed_right > 0:
        GPIO.output(BIN1, GPIO.LOW);  GPIO.output(BIN2, GPIO.HIGH)
        pwm_b.ChangeDutyCycle(speed_right)
    elif speed_right < 0:
        GPIO.output(BIN1, GPIO.HIGH); GPIO.output(BIN2, GPIO.LOW)
        pwm_b.ChangeDutyCycle(-speed_right)
    else:
        GPIO.output(BIN1, GPIO.LOW);  GPIO.output(BIN2, GPIO.LOW)
        pwm_b.ChangeDutyCycle(0)


def reset_encoders():
    global left_encoder_offset, right_encoder_offset
    global left_encoder_count, right_encoder_count
    left_encoder_offset  = raw_left_encoder
    right_encoder_offset = raw_right_encoder
    left_encoder_count   = 0
    right_encoder_count  = 0


def rotate_in_place(degrees, max_speed=28):
    """
    Rotate in place immediately after driving.
    """
    move_motors(0, 0)
    print("\n[Rotate] Waiting for momentum to fully settle...")
    time.sleep(1.5)

    reset_encoders()
    time.sleep(0.1)

    target_ticks = abs(degrees) * TICKS_PER_DEGREE_HOT
    dir_left  =  1 if degrees > 0 else -1
    dir_right = -1 if degrees > 0 else  1

    print(f"\n--- Rotating {abs(degrees)}° to face object ---")

    try:
        while True:
            avg_ticks = (abs(left_encoder_count) + abs(right_encoder_count)) / 2.0
            remaining = target_ticks - avg_ticks

            # Print feedback to terminal
            sys.stdout.write(f"\r  L:{left_encoder_count:5d}  R:{right_encoder_count:5d}  |  Avg:{avg_ticks:6.1f} / {target_ticks:.1f}   ")
            sys.stdout.flush()

            if remaining <= 0:
                print("\nTurn Complete.")
                break

            speed = max_speed
            if remaining < 40:
                speed = max(16.0, max_speed * (remaining / 40.0))

            move_motors(speed * dir_left, speed * dir_right)
            time.sleep(0.01)

    finally:
        move_motors(0, 0)
        time.sleep(0.5)


# ==========================================
# MAIN MISSION LOOP
# ==========================================
def main_loop():
    global last_error, integral, running
    
    setup_gpio()

    print("=" * 55)
    print("  Line Follower + Right Object Detector")
    print("=" * 55)
    
    # Threshold configuration
    RIGHT_OBJECT_THRESHOLD_CM = 30.0 
    
    print("Initializing Camera for Line Tracking...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    
    # Lower exposure slightly for glare if needed
    picam2.set_controls({"ExposureValue": -1.5})
    picam2.start()

    print("Mission starting in 3 seconds! Make sure robot is on the line.")
    time.sleep(3)

    try:
        object_found = False
        
        while not object_found:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]
            frame_center_x = w // 2

            left_motor_speed = 0
            right_motor_speed = 0

            # ---------------------------------------------------------
            # 1. CHECK RIGHT SIDE FOR OBJECT
            # ---------------------------------------------------------
            if 0.1 < dist_right < RIGHT_OBJECT_THRESHOLD_CM:
                print(f"\n[ALERT] Object detected on Right Side at {dist_right:.1f} cm!")
                object_found = True
                move_motors(0, 0)
                break
                
            # ---------------------------------------------------------
            # 2. EMERGENCY AVOIDANCE FRONT
            # ---------------------------------------------------------
            if dist_front < 15.0:
                cv2.putText(frame, "OBSTACLE FRONT! STOPPING", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                move_motors(0, 0)
            
            # ---------------------------------------------------------
            # 3. LINE TRACKING
            # ---------------------------------------------------------
            else:
                # Bottom half ROI
                roi_start = h // 2
                roi = frame[roi_start:h, 0:w]

                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                
                # Threshold black line
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
                            
                            integral += error
                            derivative = error - last_error
                            last_error = error

                            # PID Adjustment
                            turn_adjustment = (KP * error) + (KI * integral) + (KD * derivative)

                            # Calculate final wheel speeds
                            left_motor_speed = BASE_SPEED + turn_adjustment
                            right_motor_speed = BASE_SPEED - turn_adjustment

                            # Visuals
                            x, y, bw, bh = cv2.boundingRect(c)
                            cv2.rectangle(frame, (x, y + roi_start), (x + bw, y + bh + roi_start), (0, 255, 0), 2)
                            cv2.circle(frame, (cx, cy + roi_start), 5, (255, 0, 0), -1)
                            cv2.line(frame, (frame_center_x, roi_start), (cx, cy + roi_start), (0, 0, 255), 2)
                else:
                    cv2.putText(frame, "No Line Detected", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Apply speed
                move_motors(left_motor_speed, right_motor_speed)

            # ---------------------------------------------------------
            # 4. HUD Drawing
            # ---------------------------------------------------------
            cv2.putText(frame, f"F: {int(dist_front)}cm | L: {int(dist_left)}cm | R: {int(dist_right)}cm (Limit: {RIGHT_OBJECT_THRESHOLD_CM})", 
                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"PWM L:{int(left_motor_speed)}  R:{int(right_motor_speed)}", 
                        (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Robot AI View", frame)

            if cv2.waitKey(1) == ord('q'):
                print("Quit pressed.")
                break

        # ---------------------------------------------------------
        # POST-OBJECT DETECTION (Out of loop)
        # ---------------------------------------------------------
        if object_found:
            cv2.destroyAllWindows()  # Close camera window as we transition tasks
            print("Preparing to rotate to face object...")
            
            # Use HOT rotation because the robot was just driving/following line
            rotate_in_place(90, max_speed=28)
            print("\n========== Mission Complete ==========")

    except KeyboardInterrupt:
        print("\n\nStopped by user.")

    finally:
        running = False
        move_motors(0, 0)
        picam2.stop()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main_loop()
