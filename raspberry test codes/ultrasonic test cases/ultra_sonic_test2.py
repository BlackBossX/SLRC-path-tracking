import RPi.GPIO as GPIO
import time
import sys

GPIO.setmode(GPIO.BCM)

# ----------------------------
# USER: Double check these BCM pins! 
# Right now BCM 11 is physical pin 23. BCM 12 is physical pin 32.
# ----------------------------
TRIG_PIN = 11
ECHO_PIN = 12

GPIO.setup(TRIG_PIN, GPIO.OUT)
# Force a pull-down resistor to prevent floating ghost triggers
GPIO.setup(ECHO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.output(TRIG_PIN, GPIO.LOW)

print("Ultrasonic Sensor Continuous Test Started on BCM 11/12")
print("Press Ctrl+C to quit.\n")
print("Waiting for sensor to settle...")
time.sleep(2)

try:
    while True:
        # 1. Send Ping
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001) # 10 microseconds
        GPIO.output(TRIG_PIN, GPIO.LOW)
        
        pulse_send = time.time()
        pulse_received = time.time()
        timeout = time.time() + 0.05 # Add a 50ms timeout so a frozen sensor won't crash the script

        # 2. Wait for Echo to go HIGH
        while GPIO.input(ECHO_PIN) == 0:
            pulse_send = time.time()
            if time.time() > timeout:
                break # Break out if spinning forever

        # 3. Wait for Echo to go LOW
        while GPIO.input(ECHO_PIN) == 1:
            pulse_received = time.time()
            if time.time() > timeout:
                break 

        # 4. Calculate
        pulse_duration = pulse_received - pulse_send
        
        # If it didn't timeout (duration > 0 and reasonable), print it
        if pulse_duration > 0 and pulse_duration < 0.05:
            distance = 34300 * (pulse_duration / 2.0)
            print(f"\rDistance: {distance:.2f} cm        ", end="", flush=True)
        else:
            print(f"\rDistance: [TIMEOUT / NO SIGNAL] ", end="", flush=True)

        time.sleep(0.1) # Check 10 times a second

except KeyboardInterrupt:
    print("\nTest stopped by user.")
    
finally:
    GPIO.cleanup()

