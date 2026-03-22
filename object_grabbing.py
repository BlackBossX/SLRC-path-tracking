import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera2 import Picamera2
import sys

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

# Servos
ELBOW_PIN = 18
WRIST_PIN = 13
GRIPPER_PIN = 19

# Ultrasonics
US_FRONT_TRIG = 8
US_FRONT_ECHO = 7

# ==========================================
# GLOBAL VARIABLES
# ==========================================
pwm_a = None
pwm_b = None
pwm_elbow = None
pwm_wrist = None
pwm_gripper = None

# Keep track of servo angles to allow smooth transitions
current_elbow = 80
current_wrist = 10
current_gripper = 150

BASE_SPEED = 18
KP = 0.05  # Proportional control for tracking object

# ==========================================
# SETUP & HELPER FUNCTIONS
# ==========================================
def setup_gpio():
    global pwm_a, pwm_b, pwm_elbow, pwm_wrist, pwm_gripper

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motors
    GPIO.setup([PWMA, PWMB, AIN1, AIN2, BIN1, BIN2], GPIO.OUT)
    pwm_a = GPIO.PWM(PWMA, 1000)
    pwm_b = GPIO.PWM(PWMB, 1000)
    pwm_a.start(0)
    pwm_b.start(0)

    # Servos (50Hz)
    GPIO.setup([ELBOW_PIN, WRIST_PIN, GRIPPER_PIN], GPIO.OUT)
    pwm_elbow = GPIO.PWM(ELBOW_PIN, 50)
    pwm_wrist = GPIO.PWM(WRIST_PIN, 50)
    pwm_gripper = GPIO.PWM(GRIPPER_PIN, 50)
    
    pwm_elbow.start(0)
    pwm_wrist.start(0)
    pwm_gripper.start(0)

    # Sensors
    GPIO.setup(US_FRONT_TRIG, GPIO.OUT)
    GPIO.setup(US_FRONT_ECHO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.output(US_FRONT_TRIG, GPIO.LOW)
    time.sleep(0.1)

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

def set_servo_angle(pwm, angle):
    # Standard 50Hz servo duty cycle formula (0 to 180 degrees -> 2.5% to 12.5%)
    duty = 2.5 + (10.0 * angle / 180.0)
    pwm.ChangeDutyCycle(duty)

def set_servo_angle_smooth(pwm, start_angle, target_angle, step_delay=0.03):
    """Moves the servo smoothly by stepping 1 degree at a time with a small delay."""
    step = 1 if target_angle > start_angle else -1
    for angle in range(int(start_angle), int(target_angle) + step, step):
        set_servo_angle(pwm, angle)
        time.sleep(step_delay)
    return target_angle

def init_arm():
    global current_elbow, current_wrist, current_gripper
    print("Initializing Arm...")
    
    # Safely start at neutral before moving to folded position
    set_servo_angle(pwm_gripper, 150)
    current_gripper = 150
    set_servo_angle(pwm_wrist, 90)
    current_wrist = 90
    set_servo_angle(pwm_elbow, 90)
    current_elbow = 90
    time.sleep(1.0)

    print("Moving arm to LANDING position initially before setting off...")
    # Sequence: wrsit 180 > elbow 100 > gripper 120 > wrist 50 > elbow 150 > wrist 100
    print("Pre-Initialize: Wrist to 180...")
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 180, step_delay=0.03)
    time.sleep(0.5)
    
    print("Pre-Initialize: Elbow to 100...")
    current_elbow = set_servo_angle_smooth(pwm_elbow, current_elbow, 100, step_delay=0.03)
    time.sleep(0.5)
    
    print("Pre-Initialize: Gripper to 120...")
    current_gripper = set_servo_angle_smooth(pwm_gripper, current_gripper, 120, step_delay=0.03)
    time.sleep(0.5)
    
    print("Pre-Initialize: Wrist to 50...")
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 50, step_delay=0.03)
    time.sleep(0.5)
    
    print("Pre-Initialize: Elbow to 150...")
    current_elbow = set_servo_angle_smooth(pwm_elbow, current_elbow, 150, step_delay=0.03)
    time.sleep(0.5)

    print("Pre-Initialize: Wrist to 100 (Final Standby Shape)...")
    current_wrist = set_servo_angle_smooth(pwm_wrist, current_wrist, 100, step_delay=0.03)
    time.sleep(1.0)

    # Release servo holding torque after initializing
    pwm_elbow.ChangeDutyCycle(0)
    pwm_wrist.ChangeDutyCycle(0)
    pwm_gripper.ChangeDutyCycle(0)
    print("Arm initialized and servos relaxed.")

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
    
    # Sequence: wrsit 180 > elbow 100 > gripper 120 > wrist 50 > elbow 150 > wrist 100
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
    print("Landing Sequence Complete!")

# ==========================================
# MAIN LOOP
# ==========================================
def main_loop():
    setup_gpio()
    init_arm()

    print("Initializing Camera for RED object tracking...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.start()

    grab_threshold_area = 55000  # Camera area size meaning "It's close enough in front of the robot!"
    object_grabbed = False
    
    try:
        while not object_grabbed:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame.shape[:2]
            frame_center_x = w // 2

            # Convert to HSV to track RED color
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Red color range in HSV space (Wraps around hue=0/180)
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
                        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

                        cv2.putText(frame, f"Area: {int(area)}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                        # Once we can see the red object well enough, we use the Front Ultrasonic distance to know when to stop
                        front_dist = read_ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO)
                        
                        # Draw distance so you can see it on the monitor
                        cv2.putText(frame, f"Dist: {front_dist:.1f} cm", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

                        # Target distance is exactly 12cm
                        if 0 < front_dist <= 12.0:
                            # Before completely stopping to grab, verify we are CENTERED
                            error = cx - frame_center_x
                            if abs(error) > 25:
                                # We are close but NOT centered. Pivot in place until centered!
                                cv2.putText(frame, "CENTERING TO GRAB...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                                turn_adj_center = max(12, min(20, KP * abs(error) * 2)) # Ensure enough power to turn
                                if error > 0:
                                    left_motor_speed = turn_adj_center
                                    right_motor_speed = -turn_adj_center
                                else:
                                    left_motor_speed = -turn_adj_center
                                    right_motor_speed = turn_adj_center
                            else:
                                print(f"\nObject is 12cm close & CENTERED! Stopping to execute arm.")
                                move_motors(0, 0)
                                
                                # Turn off the camera feed before grabbing
                                print("Turning off camera feed...")
                                try:
                                    picam2.stop()
                                    cv2.destroyAllWindows()
                                except:
                                    pass
                                    
                                grab_object()
                                
                                # Wait a moment after grabbing securely
                                time.sleep(2.0)
                                
                                # Proceed with Landing immediately after for testing
                                land_object()
                                
                                object_grabbed = True
                                break

                        # PID tracking to align dead-center while driving towards it
                        if not object_grabbed and (front_dist > 12.0 or front_dist < 0):
                            error = cx - frame_center_x
                        turn_adj = KP * error
                        
                        left_motor_speed = BASE_SPEED + turn_adj
                        right_motor_speed = BASE_SPEED - turn_adj
                        
                        cv2.putText(frame, "TRACKING RED OBJECT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    cv2.putText(frame, "SEARCHING (Red too small)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # Basic scanning to find the red block if it's not visible
                cv2.putText(frame, "SEARCHING (Scanning)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                left_motor_speed = 15
                right_motor_speed = -15

            move_motors(left_motor_speed, right_motor_speed)
            cv2.putText(frame, f"L_PWM: {int(left_motor_speed)} | R_PWM: {int(right_motor_speed)}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Red Object Grabber", frame)
            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nInterrupted by User")

    finally:
        print("\nCleaning up...")
        move_motors(0, 0)
        # We NO LONGER set ChangeDutyCycle(0) here because we want the servos to
        # HOLD their position with power so the arm does not fall down while the script finishes!
        # The servos will stay rigid until the power is fully cut to the Pi or script completely dies.
        try:
            picam2.stop()
            cv2.destroyAllWindows()
        except:
            pass
        # Note: calling GPIO.cleanup() strips the PWM configurations and typically drops the lines, 
        # so for this specific script we can optionally omit GPIO.cleanup() if we want the hardware pin 
        # states to persist after the program ends. For now, it will exit cleanly.
        
        # Don't cleanup so that it freezes the pins high holding the servos.
        # GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main_loop()
