import RPi.GPIO as GPIO
import time
import sys

# Define Pins (BCM Numbering)
# Sensor 1
TRIG_1 = 11
ECHO_1 = 12

# Sensor 2
TRIG_2 = 23
ECHO_2 = 22

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Setup Sensor 1 with Pull-Down Resistor
    GPIO.setup(TRIG_1, GPIO.OUT)
    GPIO.setup(ECHO_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.output(TRIG_1, GPIO.LOW)

    # Setup Sensor 2 with Pull-Down Resistor
    GPIO.setup(TRIG_2, GPIO.OUT)
    GPIO.setup(ECHO_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.output(TRIG_2, GPIO.LOW)

    print("Waiting for sensors to settle...")
    time.sleep(1)

def get_distance(trig_pin, echo_pin):
    # 1. Send 10 microsecond Ping
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    start_time = time.time()
    stop_time = time.time()
    
    timeout = time.time() + 0.05 # 50ms timeout so a broken wire/sensor won't freeze the Pi

    # 2. Wait for Echo to go HIGH
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
        if start_time > timeout:
            return -1  # Timeout Error

    # 3. Wait for Echo to go LOW again
    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()
        if stop_time > timeout:
            return -1  # Timeout Error

    # 4. Calculate Distance
    time_elapsed = stop_time - start_time
    
    # Multiply with the sonic speed (34300 cm/s) and divide by 2 (there and back)
    distance = (time_elapsed * 34300) / 2
    return distance

if __name__ == '__main__':
    try:
        setup()
        print("Dual Ultrasonic Test Started! Press Ctrl+C to stop.")
        print("-" * 50)
        
        while True:
            dist1 = get_distance(TRIG_1, ECHO_1)
            time.sleep(0.05) # 50ms delay between sensors to prevent echo waves colliding
            
            dist2 = get_distance(TRIG_2, ECHO_2)
            time.sleep(0.05) 
            
            # Formatting results
            str_d1 = f"{dist1:.1f} cm" if dist1 != -1 else "TIMEOUT"
            str_d2 = f"{dist2:.1f} cm" if dist2 != -1 else "TIMEOUT"
            
            # Print on the same line overwriting itself
            print(f"\rSensor 1 (BCM 11/12): {str_d1:<10} | Sensor 2 (BCM 23/22): {str_d2:<10}", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\nTest stopped by User.")
        GPIO.cleanup()
        sys.exit()
