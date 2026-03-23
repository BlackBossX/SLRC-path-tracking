import RPi.GPIO as GPIO
import time
import sys

# ==========================================
# HARDWARE PIN DEFINITIONS
# ==========================================
US_FRONT_TRIG = 8
US_FRONT_ECHO = 7

US_RIGHT_TRIG = 9
US_RIGHT_ECHO = 10

US_LEFT_TRIG = 23
US_LEFT_ECHO = 22

# ==========================================
# SETUP AND READ FUNCTIONS
# ==========================================
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Setup Trigger Pins as Output
    GPIO.setup([US_FRONT_TRIG, US_RIGHT_TRIG, US_LEFT_TRIG], GPIO.OUT)
    
    # Setup Echo Pins as Input with Pull-Down resistors
    GPIO.setup([US_FRONT_ECHO, US_RIGHT_ECHO, US_LEFT_ECHO], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # Ensure all triggers are LOW initially
    GPIO.output([US_FRONT_TRIG, US_RIGHT_TRIG, US_LEFT_TRIG], GPIO.LOW)
    print("Waiting for sensors to settle...")
    time.sleep(1)

def read_ultrasonic(trig_pin, echo_pin):
    # Fire a 10 microsecond pulse
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    pulse_send = time.time()
    pulse_received = time.time()
    timeout = time.time() + 0.05  # 50ms timeout to prevent locking up

    # Wait for the echo to go HIGH (Pulse starts)
    while GPIO.input(echo_pin) == 0:
        pulse_send = time.time()
        if time.time() > timeout:
            return 999.0

    # Wait for the echo to go LOW (Pulse ends)
    while GPIO.input(echo_pin) == 1:
        pulse_received = time.time()
        if time.time() > timeout:
            return 999.0

    # Calculate distance based on speed of sound (34300 cm/s)
    pulse_duration = pulse_received - pulse_send
    
    # Ignore crazy reflections (max 38ms flight time represents ~6.5m)
    if 0 < pulse_duration < 0.038: 
        return 34300 * (pulse_duration / 2.0)
    
    return 999.0

# ==========================================
# MAIN LOOP
# ==========================================
if __name__ == "__main__":
    setup()
    print("Raspberry Pi - Multi-Ultrasonic Sensor Test")
    print("Press Ctrl+C to stop...\n")
    
    try:
        while True:
            # We add a tiny delay between readings so the ultrasonic "chirps" don't mix and cause false readings
            front_dist = read_ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO)
            time.sleep(0.05)
            
            left_dist = read_ultrasonic(US_LEFT_TRIG, US_LEFT_ECHO)
            time.sleep(0.05)
            
            right_dist = read_ultrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO)
            time.sleep(0.05)
            
            # Print to same line repeatedly for a clean HUD effect
            sys.stdout.write(f"\r[FRONT: {front_dist:6.1f} cm]   |   [LEFT: {left_dist:6.1f} cm]   |   [RIGHT: {right_dist:6.1f} cm]      ")
            sys.stdout.flush()
            
    except KeyboardInterrupt:
        print("\n\nTest stopped by user.")
    finally:
        print("Cleaning up GPIO...")
        GPIO.cleanup()
        sys.exit(0)
