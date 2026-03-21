import RPi.GPIO as GPIO
import time
import sys
import serial
import threading

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

# ==========================================
# GLOBAL VARIABLES
# ==========================================
raw_left_encoder = 0
raw_right_encoder = 0
left_encoder_offset = 0
right_encoder_offset = 0
left_encoder_count = 0
right_encoder_count = 0

pwm_a = None
pwm_b = None

# --- CALIBRATION CONSTANTS (Adjust these!) ---
TICKS_PER_CM = 5.0
TICKS_PER_DEGREE_LEFT = 1.80   
TICKS_PER_DEGREE_RIGHT = 1.70  

def serial_read_thread():
    global raw_left_encoder, raw_right_encoder
    global left_encoder_count, right_encoder_count
    
    ports_to_try = ['/dev/serial0', '/dev/ttyS0', '/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyACM0']
    arduino = None
    
    for port in ports_to_try:
        try:
            arduino = serial.Serial(port, 115200, timeout=1)
            print(f"Successfully connected to Arduino on {port}")
            break
        except Exception:
            continue
            
    if arduino is None:
        print("\nERROR: Could not find Arduino on any hardware or USB port.")
        sys.exit(1)

    while True:
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

def setup_gpio():
    global pwm_a, pwm_b

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup([PWMA, PWMB, AIN1, AIN2, BIN1, BIN2], GPIO.OUT)

    pwm_a = GPIO.PWM(PWMA, 1000)
    pwm_b = GPIO.PWM(PWMB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)

    t = threading.Thread(target=serial_read_thread, daemon=True)
    t.start()
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

def reset_encoders():
    global left_encoder_offset, right_encoder_offset
    global left_encoder_count, right_encoder_count
    
    left_encoder_offset = raw_left_encoder
    right_encoder_offset = raw_right_encoder
    
    left_encoder_count = 0
    right_encoder_count = 0

def drive_distance(cm, max_speed=20):
    move_motors(0, 0)
    time.sleep(0.5)
    
    reset_encoders()
    target_ticks = abs(cm * TICKS_PER_CM)
    direction = 1 if cm > 0 else -1

    print(f"Driving {cm} cm ({target_ticks} ticks)...")
    
    try:
        while True:
            avg_ticks_driven = (abs(left_encoder_count) + abs(right_encoder_count)) / 2.0
            remaining_ticks = target_ticks - avg_ticks_driven
            
            sys.stdout.write(f"\rProgress: {avg_ticks_driven:5.1f} / {target_ticks:5.1f} ticks | L: {left_encoder_count:5} R: {right_encoder_count:5}   ")
            sys.stdout.flush()
            
            if remaining_ticks <= 0:
                print() 
                break
                
            current_speed = max_speed
            if remaining_ticks < 40:
                current_speed = max(18.0, max_speed * (remaining_ticks / 40.0))
                
            drive_speed = current_speed * direction
            move_motors(drive_speed, drive_speed)
            
            time.sleep(0.01)
    finally:
        move_motors(0, 0)
        print("Done driving.\n")
        time.sleep(0.5)


if __name__ == "__main__":
    setup_gpio()
    print("FORWARD / BACKWARD Test")
    
    print("Starting in 3 seconds...")
    time.sleep(3)
    
    try:
        travel_dist = 182.88
        
        # 1. Go Forward
        print(f"\n--- STEP 1: Driving FORWARD {travel_dist} cm ---")
        drive_distance(travel_dist, max_speed=35) 
        time.sleep(1)
        
        # 2. Go Backward
        print(f"\n--- STEP 2: Driving BACKWARD {travel_dist} cm ---")
        drive_distance(-travel_dist, max_speed=35)
        
        print("\nTest Sequence Complete!")

    except KeyboardInterrupt:
        print("\n\nTest stopped by user.")
    finally:
        move_motors(0, 0)
        GPIO.cleanup()
        sys.exit(0)
