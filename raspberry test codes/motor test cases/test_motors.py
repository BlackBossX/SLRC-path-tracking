import RPi.GPIO as GPIO
import time
import sys
import threading

# Define GPIO pins (using BCM numbering based on your specs)
# Motor A (Left)
PWMA = 12  # Hardware PWM
AIN1 = 5
AIN2 = 6

# Motor B (Right)
PWMB = 16  # Software PWM
BIN1 = 20
BIN2 = 21

# Setup
# --- Left Encoder ---
ENC_LEFT_A = 24
ENC_LEFT_B = 25

# --- Right Encoder ---
ENC_RIGHT_A = 8
ENC_RIGHT_B = 7

# Global variables to store encoder counts
left_encoder_count = 0
right_encoder_count = 0

# PWM objects
pwm_a = None
pwm_b = None

def left_encoder_isr(channel):
    global left_encoder_count
    # Read the other pin to determine direction of rotation
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

def setup():
    global pwm_a, pwm_b

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor pins setup
    GPIO.setup([PWMA, PWMB, AIN1, AIN2, BIN1, BIN2], GPIO.OUT)

    # Encoder pins setup (Inputs with pull-up resistors)
    GPIO.setup([ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Initialize PWM at 1000 Hz
    pwm_a = GPIO.PWM(PWMA, 1000)
    pwm_b = GPIO.PWM(PWMB, 1000)
    
    pwm_a.start(0)
    pwm_b.start(0)

    # Attach hardware interrupts for encoders (Triggers on RISING edge)
    GPIO.add_event_detect(ENC_LEFT_A, GPIO.RISING, callback=left_encoder_isr)
    GPIO.add_event_detect(ENC_RIGHT_A, GPIO.RISING, callback=right_encoder_isr)

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

# Thread function to continuously print encoders without blocking input
def print_encoders_loop():
    while True:
        # Move cursor to a fixed row to update without scrolling wildly,
        # or simply print it continuously. We will just print inline for simplicity, 
        # but prepend carriage return to overwrite the same line if desired.
        # Doing a simple continuous print for reliable output.
        print(f"\r[Real-Time] Left ENC: {left_encoder_count} | Right ENC: {right_encoder_count}   ", end="", flush=True)
        time.sleep(0.1)

if __name__ == "__main__":
    setup()
    
    # Start the continuous encoder printing in a background thread
    encoder_thread = threading.Thread(target=print_encoders_loop, daemon=True)
    encoder_thread.start()

    print("\n\nRaspberry Pi Motor & Encoder Test Ready!")
    print("Commands: LF, LB, RF, RB, STOP")
    print("Press Ctrl+C to quit.\n")

    try:
        while True:
            # We use python's built in input() to act like a serial monitor terminal
            cmd = input("\n[CMD] -> ").strip().upper()

            if cmd == "LF":
                print(f"\nExecuting: Left Motor FORWARD")
                move_motors(50, 0)
            elif cmd == "LB":
                print(f"\nExecuting: Left Motor BACKWARD")
                move_motors(-50, 0)
            elif cmd == "RF":
                print(f"\nExecuting: Right Motor FORWARD")
                move_motors(0, 50)
            elif cmd == "RB":
                print(f"\nExecuting: Right Motor BACKWARD")
                move_motors(0, -50)
            elif cmd == "STOP":
                print(f"\nExecuting: STOP")
                move_motors(0, 0)
            elif cmd != "":
                print(f"\nUnknown command! Try: LF, LB, RF, RB, STOP")

    except KeyboardInterrupt:
        print("\nExiting gracefully. Stopping motors...")
        move_motors(0, 0)
        GPIO.cleanup()
        sys.exit(0)

