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

    # Set encoder pins strictly as Input and enable internal Pull-Up resistors to prevent floating grounds
    GPIO.setup([ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Detect RISING
    # Removed bouncetime to prevent dropping ticks at higher speeds
    GPIO.add_event_detect(ENC_LEFT_A, GPIO.RISING, callback=left_encoder_isr)
    GPIO.add_event_detect(ENC_RIGHT_A, GPIO.RISING, callback=right_encoder_isr)

if __name__ == "__main__":
    setup()
    print("Raspberry Pi - Dual Encoder Test Started!")
    print("Spin the wheels manually by hand...")
    print("Press Ctrl+C to quit.\n")

    try:
        # Main loop simply prints the variables. 
        # The interrupts handle the counting asynchronously.
        while True:
            # \r carriage return rewrites the current line rather than making a huge scrolling list
            sys.stdout.write(f"\rLeft Encoder: {left_encoder_count:6}  |  Right Encoder: {right_encoder_count:6}")
            sys.stdout.flush()
            time.sleep(0.05) # Update 20 times a second

    except KeyboardInterrupt:
        print("\n\nTest stopped by user. Cleaning up GPIO...")
        GPIO.cleanup()
        sys.exit(0)
