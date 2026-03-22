import RPi.GPIO as GPIO
import time
import sys
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from collections import deque

# ==========================================
# HARDWARE PIN DEFINITIONS
# ==========================================
# Motors
PWMA = 12
AIN1 = 5
AIN2 = 6

PWMB = 16
BIN1 = 20
BIN2 = 21


# --- Left Encoder ---
ENC_LEFT_A = 17
ENC_LEFT_B = 27

# --- Right Encoder ---
ENC_RIGHT_A = 25
ENC_RIGHT_B = 24


# # Encoders
# ENC_LEFT_A = 14
# ENC_LEFT_B = 15
# ENC_RIGHT_A = 17
# ENC_RIGHT_B = 27

# Ultrasonics
US_RIGHT_TRIG = 9
US_RIGHT_ECHO = 10
US_LEFT_TRIG = 23
US_LEFT_ECHO = 22
US_FRONT_TRIG = 8
US_FRONT_ECHO = 7

# Servos
ELBOW_PIN = 18
WRIST_PIN = 13
GRIPPER_PIN = 19

# Stepper Motor Pins
STEP_IN1 = 2
STEP_IN2 = 3
STEP_IN3 = 11
STEP_IN4 = 26

# Stepper Sequence
step_sequence = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

# ==========================================
# CONSTANTS & GLOBALS
# ==========================================
pwm_a = None
pwm_b = None
pwm_elbow = None
pwm_wrist = None
pwm_gripper = None

left_encoder = None
right_encoder = None

current_elbow = 80
current_wrist = 10
current_gripper = 150

BASE_SPEED = 20 
KP_LINE = 0.002
KI_LINE = 0.0  
KD_LINE = 0.0005 

last_error = 0
integral = 0

KP_RED = 0.0005

TICKS_PER_DEGREE_LEFT = 0.55
TICKS_PER_DEGREE_RIGHT = 0.55

dist_front = 999.0
dist_left = 999.0
dist_right = 999.0

running = True
sensor_counter = 0

# ==========================================
# SENSOR FILTERING CLASS
# ==========================================
class UltrasonicFilter:
    def __init__(self, window_size=5, max_valid_distance=200.0, min_valid_distance=2.0):
        self.window_size = window_size
        self.max_valid = max_valid_distance
        self.min_valid = min_valid_distance
        self.readings = deque(maxlen=window_size)
        self.last_valid = 999.0
        
    def filter(self, raw_reading):
        if raw_reading < self.min_valid or raw_reading > self.max_valid:
            return self.last_valid if self.last_valid < self.max_valid else 999.0
            
        self.readings.append(raw_reading)
        
        if len(self.readings) < 2:
            self.last_valid = raw_reading
            return raw_reading
            
        sorted_readings = sorted(self.readings)
        median = sorted_readings[len(sorted_readings) // 2]
        
        if abs(median - raw_reading) > 15.0:
            filtered = median
        else:
            filtered = median * 0.7 + raw_reading * 0.3
            
        self.last_valid = filtered
        return filtered

right_filter = UltrasonicFilter(window_size=5)
left_filter = UltrasonicFilter(window_size=5)
front_filter = UltrasonicFilter(window_size=5)

# ==========================================
# ISR & SENSOR THREADS
# ==========================================
class RotaryEncoder:
    """
    Rotary Encoder class with quadrature decoding
    Handles both mechanical and optical encoders
    """
    
    def __init__(self, pin_a, pin_b, name="Encoder", invert_direction=False):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        self.invert_direction = invert_direction
        self.count = 0
        self.last_a = None
        self.last_b = None
        self.lock = threading.Lock()
        self.last_update_time = time.time()
        self.last_count = 0
        self.rpm = 0
        self.debounce_time = 2  # milliseconds
        
        # Setup GPIO pins
        GPIO.setup([pin_a, pin_b], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Read initial states
        self.last_a = GPIO.input(pin_a)
        self.last_b = GPIO.input(pin_b)
        
        # Add event detection for both pins on both edges
        GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=self._callback, bouncetime=self.debounce_time)
        GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=self._callback, bouncetime=self.debounce_time)
    
    def _callback(self, channel):
        current_a = GPIO.input(self.pin_a)
        current_b = GPIO.input(self.pin_b)
        
        with self.lock:
            # Get the state transition
            if current_a != self.last_a:
                if current_a == current_b:
                    delta = 1
                else:
                    delta = -1
                
                if self.invert_direction:
                    delta = -delta
                    
                self.count += delta
                self.last_a = current_a
                
            elif current_b != self.last_b:
                if current_a == current_b:
                    delta = -1
                else:
                    delta = 1
                
                if self.invert_direction:
                    delta = -delta
                    
                self.count += delta
                self.last_b = current_b
            
            # Update RPM
            current_time = time.time()
            if current_time - self.last_update_time >= 0.1:
                delta_count = self.count - self.last_count
                pulses_per_rev = 20
                delta_time = current_time - self.last_update_time
                if delta_time > 0:
                    self.rpm = (delta_count / pulses_per_rev) * (60 / delta_time)
                self.last_count = self.count
                self.last_update_time = current_time
    
    def get_count(self):
        with self.lock:
            return self.count
            
    def reset(self):
        with self.lock:
            self.count = 0
            self.last_count = 0
            self.last_update_time = time.time()

def reset_encoders():
    global left_encoder, right_encoder
    if left_encoder: left_encoder.reset()
    if right_encoder: right_encoder.reset()

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
    
    if pulse_duration > 0 and pulse_duration < 0.038: 
        return 34300 * (pulse_duration / 2.0)
    return 999.0

def sensor_thread_loop():
    global dist_front, dist_left, dist_right, running, sensor_counter
    
    while running:
        try:
            raw_right = read_ultrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO)
            raw_left = read_ultrasonic(US_LEFT_TRIG, US_LEFT_ECHO)
            raw_front = read_ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO)
            
            dist_right = right_filter.filter(raw_right)
            dist_left = left_filter.filter(raw_left)
            dist_front = front_filter.filter(raw_front)
            
            sensor_counter += 1
            if sensor_counter % 20 == 0: 
                print(f"[SONAR THREAD] L: {dist_left:.1f}cm | F: {dist_front:.1f}cm | R: {dist_right:.1f}cm")
                
        except Exception as e:
            pass # ignore timeouts
            
        time.sleep(0.05) 

# ==========================================
# SETUP & MOVEMENT
# ==========================================
def setup_gpio():
    global pwm_a, pwm_b, pwm_elbow, pwm_wrist, pwm_gripper, left_encoder, right_encoder

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motors
    GPIO.setup([PWMA, PWMB, AIN1, AIN2, BIN1, BIN2], GPIO.OUT)
    pwm_a = GPIO.PWM(PWMA, 1000)
    pwm_b = GPIO.PWM(PWMB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)

    # Encoders
    left_encoder = RotaryEncoder(ENC_LEFT_A, ENC_LEFT_B, name="Left")
    right_encoder = RotaryEncoder(ENC_RIGHT_A, ENC_RIGHT_B, name="Right")

    # Ultrasonics
    GPIO.setup([US_FRONT_TRIG, US_LEFT_TRIG, US_RIGHT_TRIG], GPIO.OUT)
    GPIO.setup([US_FRONT_ECHO, US_LEFT_ECHO, US_RIGHT_ECHO], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    GPIO.output(US_FRONT_TRIG, GPIO.LOW)
    GPIO.output(US_LEFT_TRIG, GPIO.LOW)
    GPIO.output(US_RIGHT_TRIG, GPIO.LOW)

    # Stepper Motor
    GPIO.setup([STEP_IN1, STEP_IN2, STEP_IN3, STEP_IN4], GPIO.OUT)
    GPIO.output(STEP_IN1, GPIO.LOW)
    GPIO.output(STEP_IN2, GPIO.LOW)
    GPIO.output(STEP_IN3, GPIO.LOW)
    GPIO.output(STEP_IN4, GPIO.LOW)
    
    # Servos
    GPIO.setup([ELBOW_PIN, WRIST_PIN, GRIPPER_PIN], GPIO.OUT)
    pwm_elbow = GPIO.PWM(ELBOW_PIN, 50)
    pwm_wrist = GPIO.PWM(WRIST_PIN, 50)
    pwm_gripper = GPIO.PWM(GRIPPER_PIN, 50)
    
    pwm_elbow.start(0)
    pwm_wrist.start(0)
    pwm_gripper.start(0)
    
    time.sleep(1)

def move_motors(speed_left, speed_right):
    speed_left = max(-100, min(100, speed_left))
    speed_right = max(-100, min(100, speed_right))

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

def rotate_in_place(degrees, max_speed=28):
    move_motors(0, 0)
    time.sleep(0.5)
    
    reset_encoders()
    
    if degrees < 0:
        target_ticks = abs(degrees * TICKS_PER_DEGREE_LEFT)
    else:
        target_ticks = abs(degrees * TICKS_PER_DEGREE_RIGHT)
        
    print(f"\n--- Rotating {degrees} degrees ({target_ticks:.1f} ticks) ---")
    
    dir_left = 1 if degrees > 0 else -1
    dir_right = -1 if degrees > 0 else 1

    try:
        while True:
            left_count = left_encoder.get_count() if left_encoder else 0
            right_count = right_encoder.get_count() if right_encoder else 0
            avg_ticks_rotated = (abs(left_count) + abs(right_count)) / 2.0
            remaining_ticks = target_ticks - avg_ticks_rotated
            
            sys.stdout.write(f"\rProgress: {avg_ticks_rotated:5.1f} / {target_ticks:5.1f} ")
            sys.stdout.flush()
            
            if remaining_ticks <= 0:
                print() 
                break
                
            current_speed = max_speed
            if remaining_ticks < 30:
                current_speed = max(18.0, max_speed * (remaining_ticks / 30.0))
                
            move_motors(current_speed * dir_left, current_speed * dir_right)
            time.sleep(0.01)
    finally:
        move_motors(0, 0)
        print("Done rotating.\n")
        time.sleep(0.5)

def move_forward_cm(target_cm, max_speed=25):
    # Approximation based on encoder specs: ~5.5 ticks per cm
    ticks_per_cm = 5.5
    target_ticks = target_cm * ticks_per_cm
    
    move_motors(0, 0)
    time.sleep(0.2)
    reset_encoders()
    
    print(f"\n--- Moving forward {target_cm} cm ({target_ticks:.1f} ticks) before turning ---")
    try:
        while True:
            left_count = left_encoder.get_count() if left_encoder else 0
            right_count = right_encoder.get_count() if right_encoder else 0
            avg_ticks = (abs(left_count) + abs(right_count)) / 2.0
            
            sys.stdout.write(f"\rFWD Progress: {avg_ticks:5.1f} / {target_ticks:5.1f} ")
            sys.stdout.flush()
            
            if avg_ticks >= target_ticks:
                print()
                break
                
            move_motors(max_speed, max_speed)
            time.sleep(0.01)
    finally:
        move_motors(0, 0)
        time.sleep(0.5)

def move_backward_cm(target_cm, max_speed=25):
    ticks_per_cm = 5.5
    target_ticks = target_cm * ticks_per_cm
    
    move_motors(0, 0)
    time.sleep(0.2)
    reset_encoders()
    
    print(f"\n--- Moving backward {target_cm} cm ({target_ticks:.1f} ticks) ---")
    try:
        while True:
            left_count = left_encoder.get_count() if left_encoder else 0
            right_count = right_encoder.get_count() if right_encoder else 0
            avg_ticks = (abs(left_count) + abs(right_count)) / 2.0
            
            sys.stdout.write(f"\rBWD Progress: {avg_ticks:5.1f} / {target_ticks:5.1f} ")
            sys.stdout.flush()
            
            if avg_ticks >= target_ticks:
                print()
                break
                
            move_motors(-max_speed, -max_speed)
            time.sleep(0.01)
    finally:
        move_motors(0, 0)
        time.sleep(0.5)

# ==========================================
# SERVO & STEPPER FUNCTIONS
# ==========================================
def set_stepper_step(step):
    GPIO.output(STEP_IN1, step[0])
    GPIO.output(STEP_IN2, step[1])
    GPIO.output(STEP_IN3, step[2])
    GPIO.output(STEP_IN4, step[3])

def rotate_stepper(steps, delay=0.002, direction=1):
    seq_len = len(step_sequence)
    step_idx = 0
    for _ in range(steps):
        set_stepper_step(step_sequence[step_idx])
        time.sleep(delay)
        step_idx += direction
        if step_idx >= seq_len:
            step_idx = 0
        elif step_idx < 0:
            step_idx = seq_len - 1
            
    # Turn off coils
    GPIO.output(STEP_IN1, GPIO.LOW)
    GPIO.output(STEP_IN2, GPIO.LOW)
    GPIO.output(STEP_IN3, GPIO.LOW)
    GPIO.output(STEP_IN4, GPIO.LOW)

def set_servo_angle(pwm, angle):
    duty = 2.5 + (10.0 * angle / 180.0)
    pwm.ChangeDutyCycle(duty)

def set_servo_angle_smooth(pwm, start_angle, target_angle, step_delay=0.03):
    step = 1 if target_angle > start_angle else -1
    for angle in range(int(start_angle), int(target_angle) + step, step):
        set_servo_angle(pwm, angle)
        time.sleep(step_delay)
    return target_angle

def init_arm():
    global current_elbow, current_wrist, current_gripper
    print("\nInitializing Arm to standby position...")
    
    set_servo_angle(pwm_gripper, 150)
    current_gripper = 150
    set_servo_angle(pwm_wrist, 90)
    current_wrist = 90
    set_servo_angle(pwm_elbow, 90)
    current_elbow = 90
    time.sleep(1.0)

    print("Pre-Initialize Sequence...")
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 180, step_delay=0.03)
    time.sleep(0.5)
    current_elbow = set_servo_angle_smooth(pwm_elbow, current_elbow, 100, step_delay=0.03)
    time.sleep(0.5)
    current_gripper = set_servo_angle_smooth(pwm_gripper, current_gripper, 120, step_delay=0.03)
    time.sleep(0.5)
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 50, step_delay=0.03)
    time.sleep(0.5)
    current_elbow = set_servo_angle_smooth(pwm_elbow, current_elbow, 150, step_delay=0.03)
    time.sleep(0.5)
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 100, step_delay=0.03)
    time.sleep(1.0)

    # Release servo holding torque after initializing
    pwm_elbow.ChangeDutyCycle(0)
    pwm_wrist.ChangeDutyCycle(0)
    pwm_gripper.ChangeDutyCycle(0)
    print("Arm initialized and servos relaxed.\n")
    
    print("\nRotating Stepper Motor 100 steps anti-clockwise to set initial point...")
    rotate_stepper(100, delay=0.002, direction=-1)

def grab_object():
    global current_elbow, current_wrist, current_gripper
    print("\n*** EXECUTING GRAB SEQUENCE ***")
    
    print("Moving Elbow smoothly to 120...")
    current_elbow = set_servo_angle_smooth(pwm_elbow, current_elbow, 120, step_delay=0.03)
    time.sleep(0.5)
    
    print("Moving Wrist smoothly to 5...")
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 5, step_delay=0.03)
    time.sleep(0.5)

    print("Moving Gripper safely to 80 (Holding)...")
    current_gripper = set_servo_angle_smooth(pwm_gripper, current_gripper, 80, step_delay=0.03)
    time.sleep(1.0)
    print("Grab Sequence Complete!")

def land_object():
    global current_elbow, current_wrist, current_gripper
    print("\n*** EXECUTING LANDING SEQUENCE ***")
    
    print("Moving Wrist smoothly to 180...")
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 180, step_delay=0.03)
    time.sleep(0.5)
    
    print("Moving Elbow smoothly to 100...")
    current_elbow = set_servo_angle_smooth(pwm_elbow, current_elbow, 100, step_delay=0.03)
    time.sleep(0.5)
    
    print("Opening Gripper safely to 150 (Releasing)...")
    current_gripper = set_servo_angle_smooth(pwm_gripper, current_gripper, 150, step_delay=0.03)
    time.sleep(0.5)
    
    print("Moving Wrist smoothly to 50...")
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 50, step_delay=0.03)
    time.sleep(0.5)
    
    print("Moving Elbow smoothly to 150 (Retracting)...")
    current_elbow = set_servo_angle_smooth(pwm_elbow, current_elbow, 150, step_delay=0.03)
    time.sleep(0.5)
    
    print("Moving Wrist smoothly to 100 (Final Rest)...")
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 100, step_delay=0.03)
    time.sleep(1.0)
    
    # Release servo holding torque
    pwm_elbow.ChangeDutyCycle(0)
    pwm_wrist.ChangeDutyCycle(0)
    pwm_gripper.ChangeDutyCycle(0)
    
    print("Landing Sequence Complete!")

# ==========================================
# MAIN LOOP LOGIC
# ==========================================
def get_stable_initial_distance(sensor_filter, name="sensor", samples=10, stable_threshold=5.0):
    print(f"Sampling {name} for stable initial distance...")
    readings = []
    
    for i in range(samples):
        time.sleep(0.1)
        current_reading = sensor_filter.last_valid if sensor_filter.last_valid < 999 else None
        
        if current_reading and current_reading < 150.0:
            readings.append(current_reading)
            print(f"  {name} Sample {i+1}: {current_reading:.1f} cm")
        
        if len(readings) >= 3:
            avg = sum(readings) / len(readings)
            variation = max(readings) - min(readings)
            if variation < stable_threshold:
                return avg
    
    if readings:
        return sum(readings) / len(readings)
    return None

def main_loop():
    global last_error, integral, running, dist_front

    setup_gpio()
    init_arm()

    s_thread = threading.Thread(target=sensor_thread_loop, daemon=True)
    s_thread.start()

    print("Initializing Camera...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.set_controls({"ExposureValue": -1.0})
    picam2.start()

    last_known_error = 0
    line_found_once = False
    
    initial_right_dist = None
    initial_left_dist = None
    object_found = False
    object_detected_side = None
    object_confirmation_count = 0
    need_confirmation = 3

    try:
        # ----------------------------------------------------
        # PHASE 0: FIND INITIAL HORIZONTAL LINE & ENTER PATH
        print("\n--- PHASE 0: SEEKING INITIAL LINE ---")
        line_reached = False
        while not line_reached:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]
            
            roi_start = h // 2
            roi = frame[roi_start:h, 0:w]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                x_box, y_box, bw, bh = cv2.boundingRect(c)
                
                # Check for horizontal line spanning across
                if bw > w * 0.8:
                    print("\n[TRIGGER] >>> DETECTED INITIAL HORIZONTAL LINE! TURNING LEFT INTO TRACK <<<")
                    move_motors(0, 0)
                    time.sleep(0.5)
                    move_forward_cm(8.0, max_speed=25) 
                    rotate_in_place(-90, max_speed=28)
                    line_reached = True
                    break
                    
            move_motors(BASE_SPEED, BASE_SPEED)
            cv2.imshow("Robot AI View", frame)
            if cv2.waitKey(1) == ord('q'):
                break

        # ----------------------------------------------------
        # PHASE 1: INITIAL WALL READINGS
        print("\n--- PHASE 1: SETTING INITIAL BASELINES ---")
        initial_right_dist = 55.0
        initial_left_dist = 55.0
        print(f"[INIT] ---> RIGHT BASELINE SET TO: {initial_right_dist:.1f} cm <---")
        print(f"[INIT] ---> LEFT BASELINE SET TO: {initial_left_dist:.1f} cm <---")
        time.sleep(0.5)

        print("\n--- PHASE 1 COMPLETE. STARTING MISSION ---")
        time.sleep(1)

        # ----------------------------------------------------
        # PHASE 2: LINE TRACKING UNTIL OBJECT
        print("\n--- PHASE 2: LINE TRACKING ---")
        while not object_found:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]
            frame_center_x = w // 2

            left_motor_speed = 0
            right_motor_speed = 0
            
            # Sub-check for object drops
            current_detected_side = None
            # Need to see drop in distance, but ensuring noise doesn't trigger it if distance is too small/invalid
            if 30 < dist_right  and dist_right < (initial_right_dist - 10.0):
                current_detected_side = "RIGHT"
            elif 30 < dist_left and dist_left < (initial_left_dist - 10.0):
                current_detected_side = "LEFT"

            if current_detected_side:
                object_confirmation_count += 1
                if object_confirmation_count >= need_confirmation:
                    if not object_found:
                        detected_dist = dist_right if current_detected_side == "RIGHT" else dist_left
                        initial_dist = initial_right_dist if current_detected_side == "RIGHT" else initial_left_dist
                        print(f"\n[TRIGGER] >>> DETECTED {current_detected_side} DROP! Baseline: {initial_dist:.1f}cm | Current: {detected_dist:.1f}cm <<<")
                        object_found = True
                        object_detected_side = current_detected_side
                        break
            else:
                object_confirmation_count = max(0, object_confirmation_count - 1)

            # Bottom half check
            roi_start = h // 2
            roi = frame[roi_start:h, 0:w]

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # WHITE LINE TRACKING
            _, thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            line_detected = False

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                if area > 500:
                    line_detected = True
                    line_found_once = True
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        x_box, y_box, bw, bh = cv2.boundingRect(c)
                        
                        # Check for horizontal line spanning left to right edge
                        # Since width of roi is 640 usually, bw > 500 is roughly left to right.
                        if bw > w * 0.8:
                            print("\n[TRIGGER] >>> DETECTED HORIZONTAL LINE! TURNING LEFT <<<")
                            move_motors(0, 0)
                            time.sleep(0.5)
                            move_forward_cm(8.0, max_speed=25) # push wheels over the line to pivot
                            rotate_in_place(-90, max_speed=28)
                            last_known_error = 0
                            integral = 0
                            continue

                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        error = cx - frame_center_x
                        last_known_error = error

                        integral += error
                        derivative = error - last_error
                        last_error = error

                        # Elbow Alignment
                        if error < -100:
                            cv2.putText(frame, "ALIGN LEFT", (20, 40), 1, 1.5, (0, 255, 255), 2)
                            left_motor_speed = -10
                            right_motor_speed = BASE_SPEED + 10
                        elif error > 100:
                            cv2.putText(frame, "ALIGN RIGHT", (20, 40), 1, 1.5, (0, 255, 255), 2)
                            left_motor_speed = BASE_SPEED + 10
                            right_motor_speed = -10
                        else:
                            cv2.putText(frame, "TRACKING LINE", (20, 40), 1, 1.5, (0, 255, 0), 2)
                            turn_adj = (KP_LINE * error) + (KI_LINE * integral) + (KD_LINE * derivative)
                            left_motor_speed = BASE_SPEED + turn_adj
                            right_motor_speed = BASE_SPEED - turn_adj

            if not line_detected:
                if not line_found_once:
                    left_motor_speed = BASE_SPEED
                    right_motor_speed = BASE_SPEED
                    cv2.putText(frame, "GOING FWD", (20, 40), 1, 1.5, (255, 0, 255), 2)
                elif last_known_error < -100:
                    left_motor_speed = -10
                    right_motor_speed = BASE_SPEED + 10
                    cv2.putText(frame, "SEEKING LEFT", (20, 40), 1, 1.5, (0, 165, 255), 2)
                elif last_known_error > 100:
                    left_motor_speed = BASE_SPEED + 10
                    right_motor_speed = -10
                    cv2.putText(frame, "SEEKING RIGHT", (20, 40), 1, 1.5, (0, 165, 255), 2)
                else:
                    left_motor_speed = BASE_SPEED
                    right_motor_speed = BASE_SPEED

            move_motors(left_motor_speed, right_motor_speed)
            
            cv2.imshow("Robot AI View", frame)
            if cv2.waitKey(1) == ord('q'):
                break
        
        # ----------------------------------------------------
        # PHASE 3: TURN 90 DEGREES
        print("\n--- PHASE 3: TURN 90 DEGREES ---")
        move_motors(0, 0)
        time.sleep(0.5)
        
        # Move slightly forward so rotation aligns perfectly with the object
        move_forward_cm(5.0, max_speed=25)

        if object_detected_side == "RIGHT":
            rotate_in_place(90, max_speed=28)
        elif object_detected_side == "LEFT":
            rotate_in_place(-90, max_speed=28)
        else:
            print("Warning: Object side not recognized correctly. Skipping rotation.")

        # Move 10cm backward before tracking box
        move_backward_cm(3.0, max_speed=25)
        
        # ----------------------------------------------------
        # PHASE 4: TRACK RED BOX & GRAB
        print("\n--- PHASE 4: RED OBJECT SEARCH & GRAB ---")
        object_grabbed = False
        
        while not object_grabbed:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]
            frame_center_x = w // 2

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Red color range
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            
            red_mask = mask1 + mask2
            red_mask = cv2.erode(red_mask, (5, 5), iterations=2)
            red_mask = cv2.dilate(red_mask, (5, 5), iterations=2)

            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            left_motor_speed = 0
            right_motor_speed = 0

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                if area > 800:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        x, y, bw, bh = cv2.boundingRect(c)
                        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
                        
                        f_dist = dist_front # thread-safe copy

                        if 0 < f_dist <= 17.0:
                            error = cx - frame_center_x

                            if abs(error) > 40:
                                cv2.putText(frame, "CENTERING", (20, 40), 1, 1.5, (0, 165, 255), 2)
                                turn_adj_center = 12
                                if error > 0:
                                    left_motor_speed = turn_adj_center
                                    right_motor_speed = -turn_adj_center
                                else:
                                    left_motor_speed = -turn_adj_center
                                    right_motor_speed = turn_adj_center
                            else:
                                print("\nObject is 12cm close & CENTERED! Stopping to grab.")
                                move_motors(-15, -15) # Brief brake to counter forward momentum
                                time.sleep(0.05)
                                # Force exact zero right now directly on the global PWM
                                left_motor_speed = 0
                                right_motor_speed = 0
                                move_motors(0, 0)
                                time.sleep(0.6) # Let all momentum settle before moving servos
                                rotate_in_place(-5, max_speed=28)
                                grab_object()
                                time.sleep(2.0)
                                land_object()
                                
                                # Step: Rotate stepper motor clockwise 100 steps
                                print("\nRotating Stepper Motor 100 steps clockwise...")
                                rotate_stepper(300, delay=0.002, direction=1)
                                
                                print("\nMoving backward 2cm before turning...")
                                move_backward_cm(2.0, max_speed=25)
                                
                                if object_detected_side == "LEFT":
                                    print("\nTurning 100 degrees Clockwise...")
                                    rotate_in_place(100, max_speed=28)
                                else:
                                    print("\nTurning 100 degrees Anti-Clockwise...")
                                    rotate_in_place(-100, max_speed=28)
                                
                                object_grabbed = True
                                break
                        else:
                            error = cx - frame_center_x
                            turn_adj = KP_RED * error
                            left_motor_speed = BASE_SPEED + turn_adj
                            right_motor_speed = BASE_SPEED - turn_adj
                            cv2.putText(frame, f"TRACK RED d:{f_dist:.1f}cm", (20, 40), 1, 1.5, (0, 0, 255), 2)

                else:
                    cv2.putText(frame, "SEARCHING RED", (20, 40), 1, 1.5, (0, 255, 255), 2)
            else:
                # Stop the wheels if we lose sight of the red box completely so it doesn't spin 360 degrees endlessly.
                left_motor_speed = 0
                right_motor_speed = 0
                cv2.putText(frame, "CANT SEE OBJECT - STOPPED", (20, 40), 1, 1.5, (0, 0, 255), 2)

            move_motors(left_motor_speed, right_motor_speed)
            cv2.imshow("Robot AI View", frame)
            if cv2.waitKey(1) == ord('q'):
                break

        # ----------------------------------------------------
        # PHASE 5: RETURN TO MAIN LINE AND KEEP TRACKING
        print("\n--- PHASE 5: KEEP TRACKING MOVE FORWARD ---")
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]
            frame_center_x = w // 2
            
            roi_start = h // 2
            roi = frame[roi_start:h, 0:w]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            _, thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            line_detected = False
            left_motor_speed = BASE_SPEED
            right_motor_speed = BASE_SPEED

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                
                if area > 500:
                    line_detected = True
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        
                        error = cx - frame_center_x
                        integral += error
                        derivative = error - last_error
                        last_error = error

                        if error < -100:
                            cv2.putText(frame, "ALIGN LEFT", (20, 40), 1, 1.5, (0, 255, 255), 2)
                            left_motor_speed = -10
                            right_motor_speed = BASE_SPEED + 10
                        elif error > 100:
                            cv2.putText(frame, "ALIGN RIGHT", (20, 40), 1, 1.5, (0, 255, 255), 2)
                            left_motor_speed = BASE_SPEED + 10
                            right_motor_speed = -10
                        else:
                            cv2.putText(frame, "TRACKING LINE", (20, 40), 1, 1.5, (0, 255, 0), 2)
                            turn_adj = (KP_LINE * error) + (KI_LINE * integral) + (KD_LINE * derivative)
                            left_motor_speed = BASE_SPEED + turn_adj
                            right_motor_speed = BASE_SPEED - turn_adj

            if not line_detected:
                cv2.putText(frame, "SEEKING LINE FWD", (20, 40), 1, 1.5, (255, 0, 255), 2)

            move_motors(left_motor_speed, right_motor_speed)
            cv2.imshow("Robot AI View", frame)
            if cv2.waitKey(1) == ord('q'):
                break

        print("\nMISSION COMPLETE!")

    except KeyboardInterrupt:
        print("\nInterrupted by User")

    finally:
        print("\nEnding gracefully...")
        running = False
        move_motors(0, 0)
        try:
            picam2.stop()
            cv2.destroyAllWindows()
        except:
            pass
        # Do NO GPIO.cleanup() so we retain holding torque on the cargo
        sys.exit(0)

if __name__ == "__main__":
    main_loop()
