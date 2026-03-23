import RPi.GPIO as GPIO
import time
import sys

# Define Pins (BCM Numbering)
# Adjust these to match whichever sensor you want to test!
TRIG_PIN = 23
ECHO_PIN = 22

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(TRIG_PIN, GPIO.OUT)
    # Force a pull-down resistor to prevent floating ghost triggers
    GPIO.setup(ECHO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # Initialize trigger to strictly False/Low
    GPIO.output(TRIG_PIN, GPIO.LOW)
    print("Waiting for sensor to settle...")
    time.sleep(1)

def get_distance():
    # 1. Send 10 microsecond Ping
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    stop_time = time.time()
    
    timeout = time.time() + 0.1 # 100ms timeout so a broken wire won't freeze the Pi forever!

    # 2. Wait for Echo to go HIGH
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()
        if start_time > timeout:
            return -1  # Timeout Error

    # 3. Wait for Echo to go LOW again
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()
        if stop_time > timeout:
            return -1  # Timeout Error

    # 4. Calculate Distance
    time_elapsed = stop_time - start_time
    
    # multiply with the sonic speed (34300 cm/s) and divide by 2 (there and back)
    distance = (time_elapsed * 34300) / 2
    return distance

if __name__ == '__main__':
    try:
        setup()
        print("Ultrasonic Test Started! Press Ctrl+C to stop.")
        
        while True:
            dist = get_distance()
            
            if dist == -1:
                print("Error: Sensor Timeout! Check wiring (Trig/Echo might be swapped, or missing power).")
            else:
                # We use \r to overwrite the line instead of spamming down the screen
                print(f"\rMeasured Distance: {dist:.1f} cm       ", end="", flush=True)
                
            time.sleep(0.1) # 10 readings per second

    except KeyboardInterrupt:
        print("\nTest stopped by User.")
        GPIO.cleanup()
        sys.exit()
