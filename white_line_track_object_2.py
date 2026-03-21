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

ENC_LEFT_A = 24
ENC_LEFT_B = 25
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
        # Basic validation - reject obvious errors
        if raw_reading < self.min_valid or raw_reading > self.max_valid:
            # Use last valid reading if available, otherwise return high value
            return self.last_valid if self.last_valid < self.max_valid else 999.0
            
        # Add to window
        self.readings.append(raw_reading)
        
        # If we don't have enough readings yet, use raw value
        if len(self.readings) < 2:
            self.last_valid = raw_reading
            return raw_reading
            
        # Remove outliers (simple median filter)
        sorted_readings = sorted(self.readings)
        median = sorted_readings[len(sorted_readings) // 2]
        
        # If median is far from raw, it might be a spike
        if abs(median - raw_reading) > 15.0:  # 15cm jump is suspicious
            # Use median instead (reject spike)
            filtered = median
        else:
            # Weighted average: 70% median, 30% raw (smoothing)
            filtered = median * 0.7 + raw_reading * 0.3
            
        self.last_valid = filtered
        return filtered

# ==========================================
# GLOBAL VARIABLES
# ==========================================
pwm_a = None
pwm_b = None

left_encoder_count = 0
right_encoder_count = 0

dist_front = 999.0
dist_left = 999.0
dist_right = 999.0

# Create filters for each sensor
right_filter = UltrasonicFilter(window_size=5)
left_filter = UltrasonicFilter(window_size=5)
front_filter = UltrasonicFilter(window_size=5)

# Base Motor Speed & PID
BASE_SPEED = 20 
KP = 0.002  
KI = 0.0  
KD = 0.0005 

last_error = 0
integral = 0

TICKS_PER_DEGREE_HOT = 0.58

# ==========================================
# ISR & SENSOR FUNCTIONS
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

    GPIO.setup([ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B], GPIO.IN)
    GPIO.add_event_detect(ENC_LEFT_A, GPIO.RISING, callback=left_encoder_isr, bouncetime=5)
    GPIO.add_event_detect(ENC_RIGHT_A, GPIO.RISING, callback=right_encoder_isr, bouncetime=5)

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

    # Wait for echo to start
    while GPIO.input(echo_pin) == 0:
        pulse_send = time.time()
        if time.time() > timeout:
            return 999.0

    # Wait for echo to end
    while GPIO.input(echo_pin) == 1:
        pulse_received = time.time()
        if time.time() > timeout:
            return 999.0

    pulse_duration = pulse_received - pulse_send
    
    # Calculate distance (34300 cm/s for speed of sound)
    if pulse_duration > 0 and pulse_duration < 0.038:  # Max ~650cm range
        distance = 34300 * (pulse_duration / 2.0)
        return distance
    return 999.0

running = True
sensor_counter = 0

def sensor_thread_loop():
    global dist_front, dist_left, dist_right, running, sensor_counter
    
    # Read sensors at a slower, more stable rate (10Hz)
    # Robot moves slowly, so 10 readings per second is plenty
    while running:
        try:
            # Read raw values
            raw_right = read_ultrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO)
            raw_left = read_ultrasonic(US_LEFT_TRIG, US_LEFT_ECHO)
            raw_front = read_ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO)
            
            # Apply filtering
            dist_right = right_filter.filter(raw_right)
            dist_left = left_filter.filter(raw_left)
            dist_front = front_filter.filter(raw_front)
            
            # Optional: Print debug info occasionally
            sensor_counter += 1
            if sensor_counter % 20 == 0:  # Print every ~2 seconds
                print(f"Sensor status - R:{dist_right:.1f} L:{dist_left:.1f} F:{dist_front:.1f}")
            
        except Exception as e:
            print(f"Sensor read error: {e}")
            
        # Slower reading rate (0.1s = 10Hz) - plenty for slow robot movement
        time.sleep(0.1)

def rotate_in_place(degrees, max_speed=28):
    move_motors(0, 0)
    print("\nWaiting for robot to fully stop before encoder reset...")
    time.sleep(1.5)

    reset_encoders()
    time.sleep(0.1)

    target_ticks = abs(degrees) * TICKS_PER_DEGREE_HOT
    dir_left  =  1 if degrees > 0 else -1
    dir_right = -1 if degrees > 0 else  1

    print(f"\n--- Rotating {degrees} degrees ---")
    
    while True:
        curr_l = abs(left_encoder_count)
        curr_r = abs(right_encoder_count)
        avg_ticks = (curr_l + curr_r) / 2.0
        
        if avg_ticks >= target_ticks:
            break

        move_motors(max_speed * dir_left, max_speed * dir_right)
        time.sleep(0.01)

    move_motors(0, 0)
    print("Rotation finished.")

def get_stable_initial_distance(sensor_filter, samples=10, stable_threshold=5.0):
    """Get a stable initial reading by taking multiple samples"""
    print("Sampling sensor for stable initial distance...")
    readings = []
    
    for i in range(samples):
        # Force a fresh reading by waiting and reading again
        time.sleep(0.1)
        # The sensor thread is already running, so we just wait for filtered value
        # But we need to ensure we get the latest filtered reading
        current_reading = sensor_filter.last_valid if sensor_filter.last_valid < 999 else None
        
        if current_reading and current_reading < 100.0:  # Valid reading
            readings.append(current_reading)
            print(f"  Sample {i+1}: {current_reading:.1f} cm")
        
        # Check if we have enough stable readings
        if len(readings) >= 3:
            # Check if readings are stable (variation < threshold)
            avg = sum(readings) / len(readings)
            variation = max(readings) - min(readings)
            if variation < stable_threshold:
                print(f"Stable reading achieved after {len(readings)} samples")
                return avg
    
    # If not stable, return average of what we have or None
    if readings:
        return sum(readings) / len(readings)
    return None

# ==========================================
# MAIN ROBOT LOOP
# ==========================================
def main_loop():
    setup_gpio()

    global running
    sensor_thread = threading.Thread(target=sensor_thread_loop, daemon=True)
    sensor_thread.start()

    print("Initializing Camera & Integration Logic (WHITE LINE)...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.set_controls({"ExposureValue": 0})
    picam2.start()

    global last_error, integral
    last_known_error = 0
    
    initial_wall_distance = None
    object_found = False
    object_confirmation_count = 0  # For debouncing object detection
    need_confirmation = 3  # Need 3 consecutive detections

    print("Starting Robot! Searching for Initial Wall Distance on Right...")
    time.sleep(2)  # Let sensors stabilize
    
    try:
        # Phase 1: Get stable initial reading
        while initial_wall_distance is None:
            # Wait for filtered values to stabilize
            temp_reading = get_stable_initial_distance(right_filter, samples=8)
            if temp_reading:
                initial_wall_distance = temp_reading
                print(f"\n---> RECORDED INITIAL WALL DISTANCE: {initial_wall_distance:.1f} cm <---\n")
            else:
                print("Waiting for valid sensor reading...")
                time.sleep(0.5)

        print("Initial distance acquired. Now starting Line Tracking & Object Search...\n")

        # Phase 2 & 3: Track line while checking for objects
        while not object_found:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]
            frame_center_x = w // 2

            left_motor_speed = 0
            right_motor_speed = 0
            
            # Object detection with debouncing
            # Check if distance is significantly closer than initial wall
            if dist_right < (initial_wall_distance - 8.0):
                object_confirmation_count += 1
                if object_confirmation_count >= need_confirmation:
                    if not object_found:  # Only trigger once
                        print(f"\n>>> OBJECT DETECTED! Current dist: {dist_right:.1f} cm, Initial dist: {initial_wall_distance:.1f} cm <<<")
                        object_found = True
                        break
            else:
                # Reset counter if reading is normal
                object_confirmation_count = max(0, object_confirmation_count - 1)

            # Bottom half check (Line Tracking)
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

                        # Drawing Visuals
                        x, y, bw, bh = cv2.boundingRect(c)
                        cv2.rectangle(frame, (x, y + roi_start), (x + bw, y + bh + roi_start), (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy + roi_start), 5, (255, 0, 0), -1)
                        cv2.line(frame, (frame_center_x, roi_start), (cx, cy + roi_start), (0, 0, 255), 2)

            if not line_detected:
                if last_known_error < -100:
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

            # Display filtered distance
            cv2.putText(frame, f"Initial Wall: {initial_wall_distance:.1f} cm", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"US Right: {dist_right:.1f} cm", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"L_PWM: {int(left_motor_speed)} | R_PWM: {int(right_motor_speed)}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Object Confirm: {object_confirmation_count}/{need_confirmation}", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            cv2.imshow("Robot AI View", frame)
            if cv2.waitKey(1) == ord('q'):
                break
        
        # --- OUTSIDE WHILE LOOP (If Object Was Found) ---
        if object_found:
            print("\n*** OBJECT DETECTED - STOPPING ROBOT ***")
            move_motors(0, 0)
            # Remove the rotation command as requested
            # rotate_in_place(90, max_speed=28)

    except KeyboardInterrupt:
        print("\nInterrupted by User")

    finally:
        print("\nCleaning up...")
        running = False
        move_motors(0, 0)
        picam2.stop()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main_loop()