import RPi.GPIO as GPIO
import time
import sys

# Define Encoder Pins
# --- Left Encoder ---
ENC_LEFT_A = 24
ENC_LEFT_B = 25

# --- Right Encoder ---
ENC_RIGHT_A = 8
ENC_RIGHT_B = 7

# Global variables to store encoder counts
left_encoder_count = 0
right_encoder_count = 0

def left_encoder_isr(channel):
    global left_encoder_count
    # Fallback to simple polling of B pin sinceBOTH edge trigger logic can be finicky on some Pi's
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

    # Set encoder pins as Input with internal Pull-Up resistors.
    # We explicitly declare pull-ups, which is what the Arduino INPUT_PULLUP did.
    # Note: On Raspberry Pi, the built-in pull-ups are sometimes weak (~50k ohm).
    GPIO.setup([ENC_LEFT_A, ENC_LEFT_B, ENC_RIGHT_A, ENC_RIGHT_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Revert to RISING edge detection (identical to Arduino's attachInterrupt with RISING parameter)
    # The Pi's BOTH logic sometimes drops pulses entirely if the debounce isn't perfect.
    GPIO.add_event_detect(ENC_LEFT_A, GPIO.RISING, callback=left_encoder_isr, bouncetime=1)
    GPIO.add_event_detect(ENC_RIGHT_A, GPIO.RISING, callback=right_encoder_isr, bouncetime=1)

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
