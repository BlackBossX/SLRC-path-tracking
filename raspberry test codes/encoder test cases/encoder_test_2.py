import RPi.GPIO as GPIO
import time
import sys

# Define Encoder Pins
# --- Left Encoder ---
ENC_LEFT_A = 24
ENC_LEFT_B = 25

# --- Right Encoder ---
ENC_RIGHT_A = 17
ENC_RIGHT_B = 27

# Global variables to store encoder counts
left_encoder_count = 0
right_encoder_count = 0

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

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Set encoder pins strictly as Input and enable internal Pull-Up resistors
    GPIO.setup([ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Detect RISING, no bouncetime to ensure we catch all fast ticks
    GPIO.add_event_detect(ENC_LEFT_A, GPIO.RISING, callback=left_encoder_isr)
    GPIO.add_event_detect(ENC_RIGHT_A, GPIO.RISING, callback=right_encoder_isr)

if __name__ == "__main__":
    setup()
    print("Raspberry Pi - Motor Rotation Test Started!")
    print("Press Ctrl+C to quit.\n")

    try:
        while True:
            # Print the total counts
            sys.stdout.write(f"\rLeft: {left_encoder_count:6} ticks | Right: {right_encoder_count:6} ticks")
            sys.stdout.flush()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user. Cleaning up GPIO...")
        GPIO.cleanup()
        sys.exit(0)
