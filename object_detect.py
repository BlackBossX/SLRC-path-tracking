import RPi.GPIO as GPIO
import time
import sys
import serial
import threading

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
# GLOBAL VARIABLES
# ==========================================
raw_left_encoder = 0
raw_right_encoder = 0
left_encoder_offset = 0
right_encoder_offset = 0
left_encoder_count = 0
right_encoder_count = 0

dist_front = 999.0
dist_left = 999.0
dist_right = 999.0

pwm_a = None
pwm_b = None

# Flag to safely stop threads on exit
running = True

TICKS_PER_CM = 5.0
TICKS_PER_DEGREE_LEFT  = 1.80
TICKS_PER_DEGREE_RIGHT = 1.70

# Averaged constant used for rotation calculations
TICKS_PER_DEGREE_AVG = (TICKS_PER_DEGREE_LEFT + TICKS_PER_DEGREE_RIGHT) / 2.0


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
        distance = 34300 * (pulse_duration / 2.0)
        return distance
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
# DRIVING & SETUP
# ==========================================
def setup_gpio():
    global pwm_a, pwm_b

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motors
    GPIO.setup([PWMA, PWMB, AIN1, AIN2, BIN1, BIN2], GPIO.OUT)

    # Ultrasonics
    GPIO.setup([US_FRONT_TRIG, US_LEFT_TRIG, US_RIGHT_TRIG], GPIO.OUT)
    GPIO.setup([US_FRONT_ECHO, US_LEFT_ECHO, US_RIGHT_ECHO], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.output(US_FRONT_TRIG, GPIO.LOW)
    GPIO.output(US_LEFT_TRIG,  GPIO.LOW)
    GPIO.output(US_RIGHT_TRIG, GPIO.LOW)

    pwm_a = GPIO.PWM(PWMA, 1000)
    pwm_b = GPIO.PWM(PWMB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)

    # Start threads
    t1 = threading.Thread(target=serial_read_thread, daemon=True)
    t1.start()

    t2 = threading.Thread(target=sensor_thread_loop, daemon=True)
    t2.start()

    time.sleep(1)  # Let sensors and serial connection settle


def move_motors(speed_left, speed_right):
    speed_left  = max(-100, min(100, speed_left))
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
    """
    Captures current raw encoder values as the new zero baseline.
    Always call this AFTER the robot has fully stopped moving.
    """
    global left_encoder_offset, right_encoder_offset
    global left_encoder_count, right_encoder_count

    left_encoder_offset  = raw_left_encoder
    right_encoder_offset = raw_right_encoder
    left_encoder_count   = 0
    right_encoder_count  = 0


def rotate_in_place(degrees, max_speed=28):
    """
    Rotate robot in place by the given number of degrees.
    Positive = turn right, Negative = turn left.

    Key fixes vs original:
      1. Motor is stopped and robot physically settles BEFORE reset_encoders()
         so the baseline is captured from a truly stationary position.
      2. A short sleep after reset gives the serial thread one cycle to
         deliver a fresh, clean encoder value before the loop starts.
      3. Uses the AVERAGE of TICKS_PER_DEGREE_LEFT and TICKS_PER_DEGREE_RIGHT
         for the target — both wheels are active during a turn so averaging
         is more accurate than picking one side.
      4. Ramp-down window widened to 40 ticks for a smoother stop.
    """
    # --- Step 1: Hard stop + wait for all momentum to die ---
    move_motors(0, 0)
    print("\nWaiting for robot to fully stop before rotation reset...")
    time.sleep(1.2)  # Increased from 1.0 — critical after high-speed driving

    # --- Step 2: Capture clean encoder baseline ---
    reset_encoders()
    time.sleep(0.1)  # Let serial thread deliver one fresh reading after reset

    # --- Step 3: Calculate target using averaged ticks/degree ---
    target_ticks = abs(degrees) * TICKS_PER_DEGREE_AVG

    # Positive degrees = right turn: left wheel forward, right wheel backward
    dir_left  =  1 if degrees > 0 else -1
    dir_right = -1 if degrees > 0 else  1

    print(f"\n--- Rotating {'RIGHT' if degrees > 0 else 'LEFT'} {abs(degrees)}° "
          f"| Target: {target_ticks:.1f} ticks "
          f"| Using {TICKS_PER_DEGREE_AVG:.3f} ticks/deg ---")

    try:
        while True:
            avg_ticks_rotated = (abs(left_encoder_count) + abs(right_encoder_count)) / 2.0
            remaining_ticks   = target_ticks - avg_ticks_rotated

            sys.stdout.write(
                f"\rRotating: L:{left_encoder_count:5d} R:{right_encoder_count:5d} "
                f"| Avg:{avg_ticks_rotated:6.1f}/{target_ticks:.1f} ticks   "
            )
            sys.stdout.flush()

            if remaining_ticks <= 0:
                print("\nTarget reached. Done turning.")
                break

            # Ramp down over the last 40 ticks for a smooth, accurate stop
            current_speed = max_speed
            if remaining_ticks < 40:
                current_speed = max(16.0, max_speed * (remaining_ticks / 40.0))

            move_motors(current_speed * dir_left, current_speed * dir_right)
            time.sleep(0.01)

    finally:
        move_motors(0, 0)
        time.sleep(0.5)

    # --- Calibration helper: log actual ticks so you can tune the constant ---
    actual_avg = (abs(left_encoder_count) + abs(right_encoder_count)) / 2.0
    actual_tpd = actual_avg / abs(degrees) if degrees != 0 else 0
    print(f"[Calibration] Actual ticks: L={left_encoder_count} R={right_encoder_count} "
          f"| Avg={actual_avg:.1f} | Measured ticks/deg={actual_tpd:.3f} "
          f"(current avg={TICKS_PER_DEGREE_AVG:.3f})")


def drive_until_object(cm, max_speed=30, threshold_cm=30.0):
    print(f"\n--- Driving max {cm} cm | Stop if right sensor < {threshold_cm} cm ---")
    move_motors(0, 0)
    time.sleep(0.5)
    reset_encoders()

    target_ticks = abs(cm * TICKS_PER_CM)
    direction    = 1 if cm > 0 else -1

    last_print_time = time.time()
    found_object    = False

    try:
        while True:
            avg_ticks_driven = (abs(left_encoder_count) + abs(right_encoder_count)) / 2.0
            remaining_ticks  = target_ticks - avg_ticks_driven

            if time.time() - last_print_time > 0.2:
                print(f"Front:{dist_front:6.1f} | Left:{dist_left:6.1f} | "
                      f"Right:{dist_right:6.1f} | Progress:{avg_ticks_driven:.1f}/{target_ticks:.1f}")
                last_print_time = time.time()

            # Object detection — ignore garbage 0.0 readings
            if 0.1 < dist_right < threshold_cm:
                print(f"\nObject detected on RIGHT at {dist_right:.1f} cm "
                      f"(threshold {threshold_cm} cm). Stopping.")
                found_object = True
                break

            if remaining_ticks <= 0:
                print(f"\nTarget distance of {cm} cm reached. No object detected.")
                break

            # Smooth deceleration in the last 40 ticks
            current_speed = max_speed
            if remaining_ticks < 40:
                current_speed = max(18.0, max_speed * (remaining_ticks / 40.0))

            move_motors(current_speed * direction, current_speed * direction)
            time.sleep(0.01)

    finally:
        move_motors(0, 0)
        time.sleep(0.5)

    return found_object


# ==========================================
# MAIN
# ==========================================
if __name__ == "__main__":
    setup_gpio()
    print("Multi-Sensor Object Detection & Turn Test")
    print(f"Rotation constant: {TICKS_PER_DEGREE_AVG:.3f} ticks/degree "
          f"(L={TICKS_PER_DEGREE_LEFT}, R={TICKS_PER_DEGREE_RIGHT})")
    print("Starting in 3 seconds...")
    time.sleep(3)

    try:
        object_found = drive_until_object(182.88, max_speed=30, threshold_cm=30.0)

        if object_found:
            print("Object found! Giving robot extra time to fully settle...")
            move_motors(0, 0)
            time.sleep(1.5)  # Extra settle time after driving — important for accuracy

            rotate_in_place(45, max_speed=28)

        print("\nTest Complete!")

    except KeyboardInterrupt:
        print("\n\nTest stopped by user.")

    finally:
        running = False
        move_motors(0, 0)
        GPIO.cleanup()
        sys.exit(0)