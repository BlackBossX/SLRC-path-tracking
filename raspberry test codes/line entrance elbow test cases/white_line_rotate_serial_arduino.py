import RPi.GPIO as GPIO
import time
import sys
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from collections import deque
import serial

# ==========================================
# HARDWARE PIN DEFINITIONS
# ==========================================
PWMA = 12
AIN1 = 5
AIN2 = 6

PWMB = 16
BIN1 = 20
BIN2 = 21

# Encoders come via SERIAL Arduino, so we don't need GPIO for them anymore!
# (Pins omitted for clarity)

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

# Serial Encoder Globals
raw_left_encoder = 0
raw_right_encoder = 0
left_encoder_offset = 0
right_encoder_offset = 0
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

# Rotation Constants - CALIBRATE THESE VALUES!
# Run the calibration function once and update these numbers
TICKS_PER_DEGREE_LEFT = 5.2    # Replace with your calibrated value
TICKS_PER_DEGREE_RIGHT = 5.2   # Replace with your calibrated value

running = True
sensor_counter = 0

# ==========================================
# ISR & SENSOR & SERIAL THREADS
# ==========================================
def serial_read_thread():
    global raw_left_encoder, raw_right_encoder
    global left_encoder_count, right_encoder_count, running
    
    ports_to_try = ['/dev/serial0', '/dev/ttyS0', '/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyACM0']
    arduino = None
    
    for port in ports_to_try:
        try:
            arduino = serial.Serial(port, 115200, timeout=1)
            print(f"Successfully connected to Arduino encoders on {port}")
            break
        except Exception:
            continue
            
    if arduino is None:
        print("\nERROR: Could not find Arduino. Continuing visually, but rotation will fail.")
    
    while running:
        if arduino:
            try:
                if arduino.in_waiting > 0:
                    line = arduino.readline().decode('utf-8', errors='ignore').strip()
                    if ',' in line:
                        parts = line.split(',')
                        if len(parts) == 2:
                            try:
                                raw_left_encoder = int(parts[0])
                                raw_right_encoder = int(parts[1])
                                left_encoder_count = raw_left_encoder - left_encoder_offset
                                right_encoder_count = raw_right_encoder - right_encoder_offset
                            except ValueError:
                                pass
                time.sleep(0.005)
            except Exception:
                pass
        else:
            time.sleep(1)

def reset_encoders():
    global left_encoder_offset, right_encoder_offset
    global left_encoder_count, right_encoder_count
    
    left_encoder_offset = raw_left_encoder
    right_encoder_offset = raw_right_encoder
    
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

    # Note: Removed Pi encoder GPIO setup, utilizing Serial thread instead!

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
# ADVANCED ROTATION - FIXED VERSION
# ==========================================
def rotate_in_place(degrees, max_speed=28):
    """
    Rotate the robot in place by a given number of degrees.
    Positive degrees = clockwise, negative = counter-clockwise.
    Uses accurate encoder difference measurement and proportional deceleration to zero.
    """
    # Stop any ongoing movement and wait for stability
    move_motors(0, 0)
    time.sleep(0.5)
    
    # Reset encoders to zero
    reset_encoders()
    time.sleep(0.02)  # Allow hardware to settle
    
    # Calculate target ticks using a single constant (average of both wheels)
    # This ensures consistency between target and measurement
    TICKS_PER_DEGREE = (TICKS_PER_DEGREE_LEFT + TICKS_PER_DEGREE_RIGHT) / 2.0
    target_ticks = degrees * TICKS_PER_DEGREE
    
    print(f"\n--- Rotating {degrees} degrees ({target_ticks:.1f} ticks) ---")
    print(f"    Using TICKS_PER_DEGREE: {TICKS_PER_DEGREE:.3f}")
    
    # Direction for each motor
    if degrees > 0:   # clockwise
        dir_left = 1
        dir_right = -1
    else:             # counter-clockwise
        dir_left = -1
        dir_right = 1
    
    # Control loop variables
    start_time = time.time()
    max_rotation_time = 30.0  # Safety timeout in seconds
    last_angle_ticks = 0
    stall_counter = 0
    angle_ticks = 0
    
    try:
        while True:
            # Safety timeout
            if time.time() - start_time > max_rotation_time:
                print("\n⚠️ TIMEOUT: Rotation did not complete")
                break
            
            # CRITICAL FIX: Use DIFFERENCE of encoders for accurate angle measurement
            # For pure rotation, left and right encoders move in opposite directions
            # Their difference divided by 2 gives the actual rotation angle
            angle_ticks = (left_encoder_count - right_encoder_count) / 2.0
            remaining_ticks = target_ticks - angle_ticks
            error = abs(remaining_ticks)
            
            # Check for stall (if encoder count isn't changing)
            if abs(angle_ticks - last_angle_ticks) < 0.5:
                stall_counter += 1
            else:
                stall_counter = 0
            last_angle_ticks = angle_ticks
            
            # If stalled for too long, abort
            if stall_counter > 50:  # 0.5 seconds at 10ms loop
                print("\n⚠️ STALL DETECTED: No encoder movement")
                break
            
            # Progress display
            sys.stdout.write(f"\rAngle: {angle_ticks:7.1f} / {target_ticks:7.1f} ticks | "
                           f"Remaining: {remaining_ticks:7.1f} | "
                           f"L: {left_encoder_count:6} R: {right_encoder_count:6}")
            sys.stdout.flush()
            
            # STOP CRITERIA: If within tolerance, stop immediately
            if error <= 0.5:  # 0.5 tick tolerance
                move_motors(0, 0)  # Stop motors NOW, not in finally
                print("\n✓ Target reached!")
                break
            
            # PROPORTIONAL SPEED CONTROL that goes to ZERO
            # Deceleration zone starts at 30 ticks
            if error < 30.0:
                # Linear ramp from max_speed to 0 as error goes from 30 to 0
                current_speed = max_speed * (error / 30.0)
                # Ensure minimum speed to overcome friction (but allow near-zero)
                if current_speed < 2.0 and error > 0.5:
                    current_speed = 2.0  # Minimal speed to keep moving
            else:
                current_speed = max_speed
            
            # Apply direction and set motor speeds
            move_motors(current_speed * dir_left, current_speed * dir_right)
            
            # Short sleep for control loop frequency
            time.sleep(0.01)
            
    except Exception as e:
        print(f"\n⚠️ Rotation error: {e}")
    
    finally:
        # Final safety stop
        move_motors(0, 0)
        print(f"Final position: {angle_ticks:.1f} ticks (error: {error:.1f} ticks)\n")
        time.sleep(0.5)

# ==========================================
# CALIBRATION FUNCTION
# ==========================================
def calibrate_rotation():
    """
    Calibrate ticks per degree by rotating 360 degrees.
    Run this once to determine the correct TICKS_PER_DEGREE values.
    """
    print("\n" + "="*50)
    print("ROTATION CALIBRATION")
    print("="*50)
    print("This will rotate the robot 360 degrees and measure actual rotation.")
    print("Make sure there's enough space around the robot!")
    print("Starting in 3 seconds...")
    time.sleep(3)
    
    # Reset encoders
    reset_encoders()
    time.sleep(0.5)
    
    # Use a known initial estimate
    initial_estimate = (TICKS_PER_DEGREE_LEFT + TICKS_PER_DEGREE_RIGHT) / 2.0
    print(f"\nInitial TICKS_PER_DEGREE estimate: {initial_estimate:.3f}")
    
    # Rotate 360 degrees at medium speed
    target_degrees = 360
    target_ticks_estimate = target_degrees * initial_estimate
    
    print(f"Rotating {target_degrees} degrees...")
    print(f"Estimated target ticks: {target_ticks_estimate:.1f}")
    
    # Start rotation
    dir_left = 1
    dir_right = -1
    speed = 25
    
    move_motors(speed * dir_left, speed * dir_right)
    
    # Let it rotate for estimated time
    estimated_time = target_ticks_estimate / (speed * 10)  # Rough estimate
    print(f"Rotating for approximately {estimated_time:.1f} seconds...")
    time.sleep(estimated_time + 1.0)  # Add extra second to ensure completion
    
    move_motors(0, 0)
    time.sleep(0.5)
    
    # Read actual encoder difference
    actual_ticks = abs((left_encoder_count - right_encoder_count) / 2.0)
    actual_degrees = (actual_ticks / initial_estimate) * target_degrees
    
    # Calculate corrected constant
    corrected_ticks_per_degree = actual_ticks / target_degrees
    
    print("\n" + "="*50)
    print("CALIBRATION RESULTS")
    print("="*50)
    print(f"Target degrees: {target_degrees}")
    print(f"Actual degrees measured: {actual_degrees:.1f}")
    print(f"Target ticks (estimated): {target_ticks_estimate:.1f}")
    print(f"Actual ticks measured: {actual_ticks:.1f}")
    print(f"\n✅ Corrected TICKS_PER_DEGREE: {corrected_ticks_per_degree:.3f}")
    print("\nUpdate your global variables with these values:")
    print(f"TICKS_PER_DEGREE_LEFT = {corrected_ticks_per_degree:.3f}")
    print(f"TICKS_PER_DEGREE_RIGHT = {corrected_ticks_per_degree:.3f}")
    print("="*50 + "\n")
    
    return corrected_ticks_per_degree

def get_stable_initial_distance(sensor_filter, samples=10, stable_threshold=5.0):
    print("Sampling sensor for stable initial distance...")
    readings = []
    
    for i in range(samples):
        time.sleep(0.1)
        current_reading = sensor_filter.last_valid if sensor_filter.last_valid < 999 else None
        
        if current_reading and current_reading < 100.0:
            readings.append(current_reading)
            print(f"  Sample {i+1}: {current_reading:.1f} cm")
        
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
    
    # Start Arduino serial thread (from test_nav_rotate.py)
    ser_thread = threading.Thread(target=serial_read_thread, daemon=True)
    ser_thread.start()

    print("Initializing Camera & Integration Logic (WHITE LINE)...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.set_controls({"ExposureValue": 0})
    picam2.start()

    global last_error, integral
    last_known_error = 0
    
    initial_wall_distance = None
    object_found = Falseimport RPi.GPIO as GPIO
import time
import sys
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from collections import deque
import serial

# ==========================================
# HARDWARE PIN DEFINITIONS
# ==========================================
PWMA = 12
AIN1 = 5
AIN2 = 6

PWMB = 16
BIN1 = 20
BIN2 = 21

# Encoders come via SERIAL Arduino, so we don't need GPIO for them anymore!
# (Pins omitted for clarity)

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

# Serial Encoder Globals
raw_left_encoder = 0
raw_right_encoder = 0
left_encoder_offset = 0
right_encoder_offset = 0
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

# Rotation Constants (Theoretical calculation based on 18cm track width and 5 ticks/cm)
TICKS_PER_DEGREE_LEFT = 0.885
TICKS_PER_DEGREE_RIGHT = 0.885

running = True
sensor_counter = 0

# ==========================================
# ISR & SENSOR & SERIAL THREADS
# ==========================================
def serial_read_thread():
    global raw_left_encoder, raw_right_encoder
    global left_encoder_count, right_encoder_count, running
    
    ports_to_try = ['/dev/serial0', '/dev/ttyS0', '/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyACM0']
    arduino = None
    
    for port in ports_to_try:
        try:
            arduino = serial.Serial(port, 115200, timeout=1)
            print(f"Successfully connected to Arduino encoders on {port}")
            break
        except Exception:
            continue
            
    if arduino is None:
        print("\nERROR: Could not find Arduino. Continuing visually, but rotation will fail.")
    
    while running:
        if arduino:
            try:
                if arduino.in_waiting > 0:
                    line = arduino.readline().decode('utf-8', errors='ignore').strip()
                    if ',' in line:
                        parts = line.split(',')
                        if len(parts) == 2:
                            try:
                                raw_left_encoder = int(parts[0])
                                raw_right_encoder = int(parts[1])
                                left_encoder_count = raw_left_encoder - left_encoder_offset
                                right_encoder_count = raw_right_encoder - right_encoder_offset
                            except ValueError:
                                pass
                time.sleep(0.005)
            except Exception:
                pass
        else:
            time.sleep(1)

def reset_encoders():
    global left_encoder_offset, right_encoder_offset
    global left_encoder_count, right_encoder_count
    
    left_encoder_offset = raw_left_encoder
    right_encoder_offset = raw_right_encoder
    
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

    # Note: Removed Pi encoder GPIO setup, utilizing Serial thread instead!

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

def get_stable_initial_distance(sensor_filter, samples=10, stable_threshold=5.0):
    print("Sampling sensor for stable initial distance...")
    readings = []
    
    for i in range(samples):
        time.sleep(0.1)
        current_reading = sensor_filter.last_valid if sensor_filter.last_valid < 999 else None
        
        if current_reading and current_reading < 100.0:
            readings.append(current_reading)
            print(f"  Sample {i+1}: {current_reading:.1f} cm")
        
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
    
    # Start Arduino serial thread (from test_nav_rotate.py)
    ser_thread = threading.Thread(target=serial_read_thread, daemon=True)
    ser_thread.start()

    print("Initializing Camera & Integration Logic (WHITE LINE)...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.set_controls({"ExposureValue": 0})
    picam2.start()

    global last_error, integral
    last_known_error = 0
    
    initial_wall_distance = None
    object_found = False
    object_confirmation_count = 0
    need_confirmation = 3

    print("Starting Robot! Searching for Initial Wall Distance on Right...")
    time.sleep(2)
    
    try:
        # Phase 1: Get stable initial reading
        while initial_wall_distance is None:
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
            
            # Sub-check for object drops
            if dist_right < (initial_wall_distance - 10.0):
                object_confirmation_count += 1
                if object_confirmation_count >= need_confirmation:
                    if not object_found:
                        print(f"\n>>> OBJECT DETECTED! Current dist: {dist_right:.1f} cm, Initial dist: {initial_wall_distance:.1f} cm <<<")
                        object_found = True
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

            cv2.putText(frame, f"Initial Wall: {initial_wall_distance:.1f} cm", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"US Right: {dist_right:.1f} cm", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"L_PWM: {int(left_motor_speed)} | R_PWM: {int(right_motor_speed)}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Robot AI View", frame)
            if cv2.waitKey(1) == ord('q'):
                break
        
        # --- OUTSIDE WHILE LOOP (If Object Was Found) ---
        if object_found:
            print("\n*** FULL STOP - EXECUTING 90 DEGREE TURN ***")
            picam2.stop()
            cv2.destroyAllWindows()
            
            # Using rotation logic borrowed explicitly from test_nav_rotate.py
            rotate_in_place(90, max_speed=28)

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
    object_confirmation_count = 0
    need_confirmation = 3

    print("Starting Robot! Searching for Initial Wall Distance on Right...")
    time.sleep(2)
    
    try:
        # Phase 1: Get stable initial reading
        while initial_wall_distance is None:
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
            
            # Sub-check for object drops
            if dist_right < (initial_wall_distance - 10.0):
                object_confirmation_count += 1
                if object_confirmation_count >= need_confirmation:
                    if not object_found:
                        print(f"\n>>> OBJECT DETECTED! Current dist: {dist_right:.1f} cm, Initial dist: {initial_wall_distance:.1f} cm <<<")
                        object_found = True
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

            cv2.putText(frame, f"Initial Wall: {initial_wall_distance:.1f} cm", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"US Right: {dist_right:.1f} cm", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"L_PWM: {int(left_motor_speed)} | R_PWM: {int(right_motor_speed)}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Robot AI View", frame)
            if cv2.waitKey(1) == ord('q'):
                break
        
        # --- OUTSIDE WHILE LOOP (If Object Was Found) ---
        if object_found:
            print("\n*** FULL STOP - EXECUTING 90 DEGREE TURN ***")
            picam2.stop()
            cv2.destroyAllWindows()
            
            # Using the fixed rotation function
            rotate_in_place(90, max_speed=28)

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

# ==========================================
# OPTIONAL: RUN CALIBRATION FIRST
# ==========================================
if __name__ == "__main__":
    # Uncomment the line below to run calibration first
    # calibrate_rotation()
    
    # Run the main robot loop
    main_loop()