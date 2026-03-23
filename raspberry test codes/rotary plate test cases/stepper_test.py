import RPi.GPIO as GPIO
import time
import sys

# Define Stepper Motor Pins (BCM)
IN1 = 2
IN2 = 3
IN3 = 11
IN4 = 26

# Sequence for a standard 4-phase stepper motor (e.g., 28BYJ-48 with ULN2003)
# This uses the half-step sequence for smoother rotation
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

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Set pins as output
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    
    # Initialize all pins to LOW
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def set_step(step):
    GPIO.output(IN1, step[0])
    GPIO.output(IN2, step[1])
    GPIO.output(IN3, step[2])
    GPIO.output(IN4, step[3])

def rotate(steps, delay=0.001, direction=1):
    """
    Rotates the stepper motor.
    steps: Number of steps to move
    delay: Delay between steps (controls speed, smaller = faster)
    direction: 1 for forward, -1 for reverse
    """
    seq_len = len(step_sequence)
    step_idx = 0
    
    for _ in range(steps):
        set_step(step_sequence[step_idx])
        time.sleep(delay)
        
        step_idx += direction
        if step_idx >= seq_len:
            step_idx = 0
        elif step_idx < 0:
            step_idx = seq_len - 1

def cleanup():
    # Turn off all coils to prevent motor from overheating
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.cleanup()

if __name__ == "__main__":
    setup()
    
    print("Raspberry Pi - Stepper Motor Test Started")
    print(f"Pins: IN1={IN1}, IN2={IN2}, IN3={IN3}, IN4={IN4}")
    print("Press Ctrl+C to quit at any time.\n")
    
    try:
        while True:
            # Full revolution for 28BYJ-48 is usually 4096 or 512 steps depending on gearing
            # We'll use 512 as a safe test value
            steps_input = input("Enter number of steps to rotate (e.g., 512, or -512 for reverse): ")
            
            try:
                steps_to_move = int(steps_input)
            except ValueError:
                print("Please enter a valid integer.")
                continue
                
            direction = 1 if steps_to_move >= 0 else -1
            steps_to_move = abs(steps_to_move)
            
            print(f"Rotating {'Forward' if direction == 1 else 'Reverse'} for {steps_to_move} steps...")
            rotate(steps_to_move, delay=0.002, direction=direction)
            print("Done!")
            
            # Turn off coils when resting
            cleanup()
            # Need to re-setup after cleanup to be ready for next iteration
            setup()

    except KeyboardInterrupt:
        print("\n\nTest stopped by user. Cleaning up GPIO...")
    finally:
        cleanup()
        sys.exit(0)
