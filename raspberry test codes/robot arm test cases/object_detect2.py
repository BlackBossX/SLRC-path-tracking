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
# CALIBRATION CONSTANTS
# ==========================================
TICKS_PER_CM = 5.0

# --- HOW TO TUNE THESE ---
# Run with CALIBRATION_MODE = True (bottom of file)
# Read the printed "[Calibration] Measured ticks/deg = X.XXX" after each turn
# Paste that number into the correct constant below
# Repeat until both tests print "Constant looks good"

# Cold = rotating from standstill, no prior driving
TICKS_PER_DEGREE_COLD = 1.75   # <-- update from Test 1 calibration output

# Hot = rotating after driving
TICKS_PER_DEGREE_HOT  = 0.58   # <-- update from Test 2 calibration output

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
# GPIO SETUP
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


# ==========================================
# MOTOR CONTROL
# ==========================================
def move_motors(speed_left, speed_right):
    speed_left  = max(-100, min(100, speed_left))
    speed_right = max(-100, min(100, speed_right))

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


# ==========================================
# ROTATION
# ==========================================
def rotate_in_place(degrees, max_speed=28, after_driving=False):
    """
    Rotate in place by `degrees`.
      +degrees = RIGHT turn
      -degrees = LEFT turn
    after_driving=True  -> uses TICKS_PER_DEGREE_HOT
    after_driving=False -> uses TICKS_PER_DEGREE_COLD
    """
    # Full stop — wait for momentum to die BEFORE resetting encoders
    move_motors(0, 0)
    print("\nWaiting for robot to fully stop before encoder reset...")
    time.sleep(1.5)

    reset_encoders()
    time.sleep(0.1)   # Let serial thread deliver one clean post-reset reading

    ticks_per_deg = TICKS_PER_DEGREE_HOT if after_driving else TICKS_PER_DEGREE_COLD
    target_ticks  = abs(degrees) * ticks_per_deg

    dir_left  =  1 if degrees > 0 else -1
    dir_right = -1 if degrees > 0 else  1

    mode  = "HOT  (after driving)" if after_driving else "COLD (standstill)"
    direc = "RIGHT" if degrees > 0 else "LEFT"
    print(f"\n--- Rotating {direc} {abs(degrees)}° ---")
    print(f"    Mode     : {mode}")
    print(f"    Constant : {ticks_per_deg:.4f} ticks/deg")
    print(f"    Target   : {target_ticks:.1f} ticks")

    try:
        while True:
            avg_ticks  = (abs(left_encoder_count) + abs(right_encoder_count)) / 2.0
            remaining  = target_ticks - avg_ticks

            sys.stdout.write(
                f"\r  L:{left_encoder_count:5d}  R:{right_encoder_count:5d}"
                f"  |  Avg:{avg_ticks:6.1f} / {target_ticks:.1f}   "
            )
            sys.stdout.flush()

            if remaining <= 0:
                print("\nTarget reached. Done turning.")
                break

            speed = max_speed
            if remaining < 40:
                speed = max(16.0, max_speed * (remaining / 40.0))

            move_motors(speed * dir_left, speed * dir_right)
            time.sleep(0.01)

    finally:
        move_motors(0, 0)
        time.sleep(0.5)

    # ---- Calibration report ----
    actual_avg = (abs(left_encoder_count) + abs(right_encoder_count)) / 2.0
    actual_tpd = actual_avg / abs(degrees) if degrees != 0 else 0
    diff_pct   = abs(actual_tpd - ticks_per_deg) / ticks_per_deg * 100 if ticks_per_deg else 0

    print(f"\n[Calibration] Requested        : {abs(degrees)}°")
    print(f"[Calibration] Ticks recorded   : L={left_encoder_count}  R={right_encoder_count}  Avg={actual_avg:.1f}")
    print(f"[Calibration] Measured ticks/deg = {actual_tpd:.4f}  |  Used = {ticks_per_deg:.4f}  |  Error = {diff_pct:.1f}%")

    if diff_pct > 5:
        constant_name = "TICKS_PER_DEGREE_HOT" if after_driving else "TICKS_PER_DEGREE_COLD"
        print(f"[Calibration] *** UPDATE {constant_name} = {actual_tpd:.4f} ***")
    else:
        print(f"[Calibration] Constant is within 5% tolerance — looks good!")


# ==========================================
# DRIVING
# ==========================================
def drive_until_object(cm, max_speed=30, threshold_cm=30.0):
    print(f"\n--- Driving max {cm} cm | Stop if RIGHT < {threshold_cm} cm ---")
    move_motors(0, 0)
    time.sleep(0.5)
    reset_encoders()

    target_ticks = abs(cm * TICKS_PER_CM)
    direction    = 1 if cm > 0 else -1
    found_object = False
    last_print   = time.time()

    try:
        while True:
            avg_ticks = (abs(left_encoder_count) + abs(right_encoder_count)) / 2.0
            remaining = target_ticks - avg_ticks

            if time.time() - last_print > 0.2:
                print(f"  F:{dist_front:6.1f}  L:{dist_left:6.1f}  R:{dist_right:6.1f}"
                      f"  |  {avg_ticks:.0f}/{target_ticks:.0f} ticks")
                last_print = time.time()

            if 0.1 < dist_right < threshold_cm:
                print(f"\nObject on RIGHT at {dist_right:.1f} cm. Stopping.")
                found_object = True
                break

            if remaining <= 0:
                print(f"\nReached {cm} cm. No object detected.")
                break

            speed = max_speed
            if remaining < 40:
                speed = max(18.0, max_speed * (remaining / 40.0))

            move_motors(speed * direction, speed * direction)
            time.sleep(0.01)

    finally:
        move_motors(0, 0)
        time.sleep(0.5)

    return found_object


# ==========================================
# CALIBRATION MODE
# Set CALIBRATION_MODE = True to find your
# correct TICKS_PER_DEGREE values.
# Set back to False for normal operation.
# ==========================================
CALIBRATION_MODE = False


def run_calibration():
    """
    Test 1: Cold rotation (no driving) -> tune TICKS_PER_DEGREE_COLD
    Test 2: Rotation after driving 50cm -> tune TICKS_PER_DEGREE_HOT
    Read the [Calibration] output after each test and update the constants.
    """
    print("\n========== CALIBRATION MODE ==========")
    print("Test 1: COLD rotation (standstill)")
    time.sleep(2)
    rotate_in_place(90, max_speed=28, after_driving=False)

    print("\nPausing 3 seconds before Test 2...")
    time.sleep(3)

    print("\nTest 2: Drive 50 cm then HOT rotation")
    drive_until_object(50, max_speed=30, threshold_cm=5.0)
    time.sleep(1.5)
    rotate_in_place(90, max_speed=28, after_driving=True)

    print("\n========== CALIBRATION COMPLETE ==========")
    print("Update TICKS_PER_DEGREE_COLD and TICKS_PER_DEGREE_HOT")
    print("at the top of this file with the values printed above.")
    print("Then set CALIBRATION_MODE = False and run normally.")


# ==========================================
# MAIN
# ==========================================
if __name__ == "__main__":
    setup_gpio()

    print("=" * 55)
    print("  Object Detection & Rotation Controller")
    print("=" * 55)
    print(f"  COLD constant : {TICKS_PER_DEGREE_COLD:.4f} ticks/deg")
    print(f"  HOT  constant : {TICKS_PER_DEGREE_HOT:.4f} ticks/deg")
    print(f"  Mode          : {'CALIBRATION' if CALIBRATION_MODE else 'NORMAL OPERATION'}")
    print("=" * 55)
    print("Starting in 3 seconds...")
    time.sleep(3)

    try:
        if CALIBRATION_MODE:
            run_calibration()
        else:
            # Normal operation
            object_found = drive_until_object(182.88, max_speed=30, threshold_cm=30.0)

            if object_found:
                print("\nObject found! Settling before rotation...")
                move_motors(0, 0)
                time.sleep(1.5)
                rotate_in_place(90, max_speed=28, after_driving=True)

            print("\n========== Test Complete! ==========")

    except KeyboardInterrupt:
        print("\n\nStopped by user.")

    finally:
        running = False
        move_motors(0, 0)
        GPIO.cleanup()
        sys.exit(0)