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
PWMA = 12
AIN1 = 5
AIN2 = 6

PWMB = 16
BIN1 = 20
BIN2 = 21

# --- Left Encoder ---
ENC_LEFT_A = 14
ENC_LEFT_B = 15

# --- Right Encoder ---
ENC_RIGHT_A = 17
ENC_RIGHT_B = 27

US_RIGHT_TRIG = 9
US_RIGHT_ECHO = 10
US_LEFT_TRIG = 23
US_LEFT_ECHO = 22
US_FRONT_TRIG = 8
US_FRONT_ECHO = 7

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

# ==========================================
# GLOBAL VARIABLES
# ==========================================
pwm_a = None
pwm_b = None

# Encoder Globals
left_encoder_count = 0
right_encoder_count = 0

dist_front = 999.0
dist_left = 999.0
dist_right = 999.0

right_filter = UltrasonicFilter(window_size=5)
left_filter = UltrasonicFilter(window_size=5)
front_filter = UltrasonicFilter(window_size=5)

BASE_SPEED = 20 
KP = 0.002  
KI = 0.0  
KD = 0.0005 

last_error = 0
integral = 0

# Rotation Constants from test_nav_rotate.py 
TICKS_PER_DEGREE_LEFT = 0.75   
TICKS_PER_DEGREE_RIGHT = 0.75  

running = True
sensor_counter = 0

# ==========================================
# ISR & SENSOR THREADS
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

def reset_encoders():
    global left_encoder_count, right_encoder_count
    left_encoder_count = 0
    right_encoder_count = 0

def setup_gpio():
    global pwm_a, pwm_b

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup([PWMA, PWMB, AIN1, AIN2, BIN1, BIN2], GPIO.OUT)
    pwm_a = GPIO.PWM(PWMA, 1000)
    pwm_b = GPIO.PWM(PWMB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)

    # Setup Encoder Pins
    GPIO.setup([ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENC_LEFT_A, GPIO.RISING, callback=left_encoder_isr)
    GPIO.add_event_detect(ENC_RIGHT_A, GPIO.RISING, callback=right_encoder_isr)

    GPIO.setup([US_FRONT_TRIG, US_LEFT_TRIG, US_RIGHT_TRIG], GPIO.OUT)
    GPIO.setup([US_FRONT_ECHO, US_LEFT_ECHO, US_RIGHT_ECHO], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    GPIO.output(US_FRONT_TRIG, GPIO.LOW)
    GPIO.output(US_LEFT_TRIG, GPIO.LOW)
    GPIO.output(US_RIGHT_TRIG, GPIO.LOW)
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
                print(f"HUD Status - R:{dist_right:.1f} L:{dist_left:.1f}")
                
        except Exception as e:
            print(f"Sensor read error: {e}")
            
        time.sleep(0.1)

# ==========================================
# ADVANCED ROTATION FROM TEST_NAV_ROTATE
# ==========================================
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
            avg_ticks_rotated = (abs(left_encoder_count) + abs(right_encoder_count)) / 2.0
            remaining_ticks = target_ticks - avg_ticks_rotated
            
            sys.stdout.write(f"\rProgress: {avg_ticks_rotated:5.1f} / {target_ticks:5.1f} ticks | L: {left_encoder_count:5} R: {right_encoder_count:5}   ")
            sys.stdout.flush()
            
            if remaining_ticks <= 0:
                print() 
                break
                
            # Slow down as it nears completion
            current_speed = max_speed
            if remaining_ticks < 30:
                current_speed = max(18.0, max_speed * (remaining_ticks / 30.0))
                
            move_motors(current_speed * dir_left, current_speed * dir_right)
            time.sleep(0.01)
    finally:
        move_motors(0, 0)
        print("Done rotating.\n")
        time.sleep(0.5)

def get_stable_initial_distance(sensor_filter, name="sensor", samples=10, stable_threshold=5.0):
    print(f"Sampling {name} for stable initial distance...")
    readings = []
    
    for i in range(samples):
        time.sleep(0.1)
        current_reading = sensor_filter.last_valid if sensor_filter.last_valid < 999 else None
        
        # Increased to 150 to allow detecting slightly further walls if placed differently
        if current_reading and current_reading < 150.0:
            readings.append(current_reading)
            print(f"  {name} Sample {i+1}: {current_reading:.1f} cm")
        
        if len(readings) >= 3:
            avg = sum(readings) / len(readings)
            variation = max(readings) - min(readings)
            if variation < stable_threshold:
                print(f"Stable reading achieved after {len(readings)} samples")
                return avg
    
    if readings:
        return sum(readings) / len(readings)
    return None

# ==========================================
# MAIN ROBOT LOOP
# ==========================================
def main_loop():
    setup_gpio()

    global running
    # Start ultrasonic thread
    s_thread = threading.Thread(target=sensor_thread_loop, daemon=True)
    s_thread.start()

    print("Initializing Camera & Integration Logic (WHITE LINE)...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.set_controls({"ExposureValue": 0})
    picam2.start()

    global last_error, integral
    last_known_error = 0
    line_found_once = False
    
    initial_right_dist = None
    initial_left_dist = None
    object_found = False
    object_detected_side = None
    object_confirmation_count = 0
    need_confirmation = 3

    print("Starting Robot! Searching for Initial Wall Distances on Both Sides...")
    time.sleep(2)
    
    try:
        # Phase 1: Get stable initial readings
        while initial_right_dist is None:
            temp_reading = get_stable_initial_distance(right_filter, name="RIGHT", samples=8)
            if temp_reading:
                initial_right_dist = temp_reading
                print(f"\n---> RECORDED INITIAL RIGHT WALL DISTANCE: {initial_right_dist:.1f} cm <---\n")
            else:
                print("Waiting for valid RIGHT sensor reading...")
                time.sleep(0.5)

        while initial_left_dist is None:
            temp_reading = get_stable_initial_distance(left_filter, name="LEFT", samples=8)
            if temp_reading:
                initial_left_dist = temp_reading
                print(f"\n---> RECORDED INITIAL LEFT WALL DISTANCE: {initial_left_dist:.1f} cm <---\n")
            else:
                print("Waiting for valid LEFT sensor reading...")
                time.sleep(0.5)

        print("Initial distances acquired. Now starting Line Tracking & Object Search...\n")

        # Phase 2 & 3: Track line while checking for objects
        while not object_found:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]
            frame_center_x = w // 2

            left_motor_speed = 0
            right_motor_speed = 0
            
            # Sub-check for object drops
            current_detected_side = None
            if dist_right < (initial_right_dist - 10.0):
                current_detected_side = "RIGHT"
            elif dist_left < (initial_left_dist - 10.0):
                current_detected_side = "LEFT"

            if current_detected_side:
                object_confirmation_count += 1
                if object_confirmation_count >= need_confirmation:
                    if not object_found:
                        detected_dist = dist_right if current_detected_side == "RIGHT" else dist_left
                        initial_dist = initial_right_dist if current_detected_side == "RIGHT" else initial_left_dist
                        print(f"\n>>> OBJECT DETECTED ON {current_detected_side}! Current dist: {detected_dist:.1f} cm, Initial dist: {initial_dist:.1f} cm <<<")
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
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        error = cx - frame_center_x
                        last_known_error = error

                        integral += error
                        derivative = error - last_error
                        last_error = error

                        # Elbow Alignment
                        if error < -100:
                            cv2.putText(frame, "ALIGNING LEFT (ELBOW)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            left_motor_speed = -10
                            right_motor_speed = BASE_SPEED + 10
                        elif error > 100:
                            cv2.putText(frame, "ALIGNING RIGHT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            left_motor_speed = BASE_SPEED + 10
                            right_motor_speed = -10
                        else:
                            cv2.putText(frame, "TRACKING WHITE LINE", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            turn_adjustment = (KP * error) + (KI * integral) + (KD * derivative)
                            left_motor_speed = BASE_SPEED + turn_adjustment
                            right_motor_speed = BASE_SPEED - turn_adjustment

                        x, y, bw, bh = cv2.boundingRect(c)
                        cv2.rectangle(frame, (x, y + roi_start), (x + bw, y + bh + roi_start), (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy + roi_start), 5, (255, 0, 0), -1)
                        cv2.line(frame, (frame_center_x, roi_start), (cx, cy + roi_start), (0, 0, 255), 2)

            if not line_detected:
                if not line_found_once:
                    cv2.putText(frame, "START - GOING FORWARD", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                    left_motor_speed = BASE_SPEED
                    right_motor_speed = BASE_SPEED
                elif last_known_error < -100:
                    cv2.putText(frame, "LOST - SEEKING LEFT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    left_motor_speed = -10
                    right_motor_speed = BASE_SPEED + 10
                elif last_known_error > 100:
                    cv2.putText(frame, "LOST - SEEKING RIGHT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    left_motor_speed = BASE_SPEED + 10
                    right_motor_speed = -10
                else:
                    cv2.putText(frame, "NO LINE - SEEKING FORWARD", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    left_motor_speed = BASE_SPEED
                    right_motor_speed = BASE_SPEED

            move_motors(left_motor_speed, right_motor_speed)

            cv2.putText(frame, f"Initial Wall R:{initial_right_dist:.1f}  L:{initial_left_dist:.1f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"US Dist R:{dist_right:.1f}  L:{dist_left:.1f}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"L_PWM: {int(left_motor_speed)} | R_PWM: {int(right_motor_speed)}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Robot AI View", frame)
            if cv2.waitKey(1) == ord('q'):
                break
        
        # --- OUTSIDE WHILE LOOP (If Object Was Found) ---
        if object_found:
            print(f"\n*** FULL STOP - EXECUTING 90 DEGREE TURN ({object_detected_side}) ***")
            picam2.stop()
            cv2.destroyAllWindows()
            
            # Using rotation logic based on which side the object was found
            if object_detected_side == "RIGHT":
                rotate_in_place(90, max_speed=28)
            elif object_detected_side == "LEFT":
                rotate_in_place(-90, max_speed=28)

    except KeyboardInterrupt:
        print("\nInterrupted by User")

    finally:
        print("\nCleaning up...")
        running = False
        move_motors(0, 0)
        try:
            picam2.stop()
            cv2.destroyAllWindows()
        except:
            pass
        GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main_loop()
