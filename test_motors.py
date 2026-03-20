import RPi.GPIO as GPIO
import time
import sys

# Define GPIO pins (using BCM numbering based on your specs)
# Motor A (Left)
PWMA = 12  # Hardware PWM
AIN1 = 5
AIN2 = 6

# Motor B (Right)
PWMB = 16  # Software PWM
BIN1 = 20
BIN2 = 21

# Encoders
# Left Motor 
ENC_A = 24
ENC_B = 25

# Right Motor
ENC_C = 8
ENC_D = 7

# Global variables to store encoder counts
left_encoder_count = 0
right_encoder_count = 0

# PWM objects
pwm_a = None
pwm_b = None

def left_encoder_isr(channel):
    global left_encoder_count
    # Read the other pin to determine direction of rotation
    if GPIO.input(ENC_B) == GPIO.HIGH:
        left_encoder_count += 1
    else:
        left_encoder_count -= 1

def right_encoder_isr(channel):
    global right_encoder_count
    if GPIO.input(ENC_D) == GPIO.HIGH:
        right_encoder_count += 1
    else:
        right_encoder_count -= 1

def setup():
    global pwm_a, pwm_b

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor pins setup
    GPIO.setup([PWMA, PWMB, AIN1, AIN2, BIN1, BIN2], GPIO.OUT)

    # Encoder pins setup (Inputs with pull-up resistors)
    GPIO.setup([ENC_A, ENC_B, ENC_C, ENC_D], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Initialize PWM at 1000 Hz
    pwm_a = GPIO.PWM(PWMA, 1000)
    pwm_b = GPIO.PWM(PWMB, 1000)
    
    pwm_a.start(0)
    pwm_b.start(0)

    # Attach hardware interrupts for encoders (Triggers on RISING edge)
    GPIO.add_event_detect(ENC_A, GPIO.RISING, callback=left_encoder_isr)
    GPIO.add_event_detect(ENC_C, GPIO.RISING, callback=right_encoder_isr)

def move_motors(speed_left, speed_right):
    """
    speed_left and speed_right should be between -100 and 100
    (representing duty cycle percentages).
    """
    # Clamp speeds to -100 to 100
    speed_left = max(-100, min(100, speed_left))
    speed_right = max(-100, min(100, speed_right))

    # --- LEFT MOTOR ---
    if speed_left > 0:
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(AIN2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(speed_left)
    elif speed_left < 0:
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.HIGH)
        pwm_a.ChangeDutyCycle(-speed_left)  # Duty cycle must be positive
    else:
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.LOW)
        pwm_a.ChangeDutyCycle(0)

    # --- RIGHT MOTOR ---
    if speed_right > 0:
        GPIO.output(BIN1, GPIO.HIGH)
        GPIO.output(BIN2, GPIO.LOW)
        pwm_b.ChangeDutyCycle(speed_right)
    elif speed_right < 0:
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.HIGH)
        pwm_b.ChangeDutyCycle(-speed_right)
    else:
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.LOW)
        pwm_b.ChangeDutyCycle(0)


if __name__ == "__main__":
    setup()
    print("Raspberry Pi Motor & Encoder Test Ready!")
    print("Commands:")
    print("  LF   - Left Motor Forward")
    print("  LB   - Left Motor Backward")
    print("  RF   - Right Motor Forward")
    print("  RB   - Right Motor Backward")
    print("  STOP - Stop all motors")
    print("  ENC  - Print current encoder counts")
    print("Press Ctrl+C to quit.")

    try:
        while True:
            # We use python's built in input() to act like a serial monitor terminal
            cmd = input("\nEnter command: ").strip().upper()

            if cmd == "LF":
                print("Executing: Left Motor FORWARD (50% Speed)")
                move_motors(50, 0)
            elif cmd == "LB":
                print("Executing: Left Motor BACKWARD (50% Speed)")
                move_motors(-50, 0)
            elif cmd == "RF":
                print("Executing: Right Motor FORWARD (50% Speed)")
                move_motors(0, 50)
            elif cmd == "RB":
                print("Executing: Right Motor BACKWARD (50% Speed)")
                move_motors(0, -50)
            elif cmd == "STOP":
                print("Executing: STOP")
                move_motors(0, 0)
            elif cmd == "ENC":
                print(f"Current Encoders -> Left: {left_encoder_count} | Right: {right_encoder_count}")
            else:
                print("Unknown command! Try: LF, LB, RF, RB, STOP, or ENC")

    except KeyboardInterrupt:
        print("\nExiting gracefully. Stopping motors...")
        move_motors(0, 0)
        GPIO.cleanup()
        sys.exit(0)
