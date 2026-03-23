import serial
import time
import sys

def main():
    # Try hardware TX/RX serial ports first, then fallback to USB options
    ports_to_try = ['/dev/serial0', '/dev/ttyS0', '/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyACM0']
    arduino = None

    print("Searching for Arduino serial connection...")
    for port in ports_to_try:
        try:
            arduino = serial.Serial(port, 115200, timeout=1)
            print(f"Successfully connected to Arduino on {port}!")
            break
        except Exception:
            continue

    if arduino is None:
        print("\nERROR: Could not find Arduino.")
        print("1. If using TX/RX, ensure raspi-config has Serial Hardware enabled.")
        print("2. If using USB, ensure the cable is plugged in.")
        sys.exit(1)

    print("\nReading Encoder Data... (Press Ctrl+C to quit)\n")
    
    # Give the connection a fraction of a second to settle
    time.sleep(0.5)

    try:
        while True:
            # Wait until there is data in the serial buffer
            if arduino.in_waiting > 0:
                # Read the line, decode it from bytes to string, and remove extra newlines/spaces
                line = arduino.readline().decode('utf-8', errors='ignore').strip()
                
                # We expect the format "LeftValue,RightValue" e.g., "1050,-400"
                if ',' in line:
                    parts = line.split(',')
                    if len(parts) == 2:
                        try:
                            left_val = int(parts[0])
                            right_val = int(parts[1])
                            
                            # Print it smoothly on one line using \r (carriage return)
                            sys.stdout.write(f"\rLeft Encoder: {left_val:>8}   |   Right Encoder: {right_val:>8}")
                            sys.stdout.flush()
                        except ValueError:
                            # Ignored in case the Arduino sent half a string during startup
                            pass
            
            # Tiny sleep to prevent maxing out the CPU
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user. Closing serial connection.")
    finally:
        if arduino and arduino.is_open:
            arduino.close()
        sys.exit(0)

if __name__ == "__main__":
    main()
