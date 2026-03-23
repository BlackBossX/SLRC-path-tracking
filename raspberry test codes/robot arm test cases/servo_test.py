import RPi.GPIO as GPIO
import time
import sys

# Define Servo Pins (GPIO BCM)
ELBOW_PIN = 18
WRIST_PIN = 13
GRIPPER_PIN = 19

def angle_to_duty_cycle(angle):
    # Standard servo 50Hz (20ms period):
    # 0 degrees   -> ~0.5ms pulse -> ~2.5% duty cycle
    # 180 degrees -> ~2.5ms pulse -> ~12.5% duty cycle
    # Map angle (0 to 180) to duty cycle (2.5 to 12.5)
    return (angle / 18.0) + 2.5

def move_servo_smoothly(pwm, start_angle, end_angle, step_delay=0.03):
    start = int(start_angle)
    end = int(end_angle)
    if start == end:
        return
    step = 1 if end > start else -1
    for angle in range(start, end + step, step):
        duty = angle_to_duty_cycle(angle)
        pwm.ChangeDutyCycle(duty)
        time.sleep(step_delay)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Set pins as output
    GPIO.setup(ELBOW_PIN, GPIO.OUT)
    GPIO.setup(WRIST_PIN, GPIO.OUT)
    GPIO.setup(GRIPPER_PIN, GPIO.OUT)

    # Initialize PWM at 50Hz (standard for servos)
    pwm_elbow = GPIO.PWM(ELBOW_PIN, 50)
    pwm_wrist = GPIO.PWM(WRIST_PIN, 50)
    pwm_gripper = GPIO.PWM(GRIPPER_PIN, 50)
    
    # Start with 0 duty cycle to prevent sudden jerky movements (servo won't move until told)
    pwm_elbow.start(0)
    pwm_wrist.start(0)
    pwm_gripper.start(0)
    
    return pwm_elbow, pwm_wrist, pwm_gripper

if __name__ == "__main__":
    pwm_elbow, pwm_wrist, pwm_gripper = setup()
    
    print("Raspberry Pi - Servo Arm Interactive Test")
    print("Control your Elbow (18), Wrist (13), and Gripper (19).")
    print("Press Ctrl+C to quit at any time.\n")
    
    # Store assumed initial angles so subsequent movements can be smooth
    current_angles = {'1': 90, '2': 90, '3': 90}
    
    try:
        while True:
            print("-" * 30)
            print("Select a joint to move:")
            print("1. Elbow   (Pin 18)")
            print("2. Wrist   (Pin 13)")
            print("3. Gripper (Pin 19)")
            
            choice = input("Enter choice (1/2/3): ").strip()
            if choice not in ['1', '2', '3']:
                print("Invalid choice. Please enter 1, 2, or 3.")
                continue
                
            angle_str = input("Enter angle (0 to 180): ").strip()
            try:
                angle = float(angle_str)
                if angle < 0 or angle > 180:
                    print("Error: Angle must be between 0 and 180.")
                    continue
            except ValueError:
                print("Error: Please enter a valid number.")
                continue
                
            duty = angle_to_duty_cycle(angle)
            
            if choice == '1':
                print(f"Moving Elbow slowly to {angle} degrees...")
                move_servo_smoothly(pwm_elbow, current_angles['1'], angle, step_delay=0.03)
                current_angles['1'] = angle
            elif choice == '2':
                print(f"Moving Wrist very slowly to {angle} degrees...")
                move_servo_smoothly(pwm_wrist, current_angles['2'], angle, step_delay=0.08)
                current_angles['2'] = angle
            elif choice == '3':
                print(f"Moving Gripper slowly to {angle} degrees...")
                move_servo_smoothly(pwm_gripper, current_angles['3'], angle, step_delay=0.03)
                current_angles['3'] = angle
                
            # Give the servo time to finish settling
            time.sleep(0.2)
            
            # Stop sending PWM pulses for Wrist and Gripper to prevent jitter.
            # (Elbow is specifically omitted so it holds its position securely with continuous power)
            if choice == '2': pwm_wrist.ChangeDutyCycle(0)
            if choice == '3': pwm_gripper.ChangeDutyCycle(0)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user. Cleaning up GPIO...")
    finally:
        # Proper cleanup
        pwm_elbow.stop()
        pwm_wrist.stop()
        pwm_gripper.stop()
        GPIO.cleanup()
        sys.exit(0)
