#!/usr/bin/env python3
"""
Robust Dual Encoder Reader for Raspberry Pi
Supports quadrature encoders with proper direction detection
"""

import RPi.GPIO as GPIO
import time
import sys
import threading
from datetime import datetime

class RotaryEncoder:
    """
    Rotary Encoder class with quadrature decoding
    Handles both mechanical and optical encoders
    """
    
    def __init__(self, pin_a, pin_b, name="Encoder", invert_direction=False):
        """
        Initialize rotary encoder
        
        Args:
            pin_a: GPIO pin number for channel A
            pin_b: GPIO pin number for channel B
            name: Name for this encoder (for debugging)
            invert_direction: If True, invert the counting direction
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        self.invert_direction = invert_direction
        self.count = 0
        self.last_a = None
        self.last_b = None
        self.lock = threading.Lock()
        self.last_update_time = time.time()
        self.last_count = 0
        self.rpm = 0
        self.debounce_time = 2  # milliseconds
        
        # Setup GPIO pins
        GPIO.setup([pin_a, pin_b], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Read initial states
        self.last_a = GPIO.input(pin_a)
        self.last_b = GPIO.input(pin_b)
        
        # Add event detection for both pins on both edges
        GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=self._callback, bouncetime=self.debounce_time)
        GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=self._callback, bouncetime=self.debounce_time)
        
        print(f"[{self.name}] Initialized on pins {pin_a} (A) and {pin_b} (B)")
    
    def _callback(self, channel):
        """
        Interrupt callback for encoder state changes
        Uses quadrature decoding algorithm
        """
        current_a = GPIO.input(self.pin_a)
        current_b = GPIO.input(self.pin_b)
        
        with self.lock:
            # Get the state transition
            if current_a != self.last_a:
                # A changed state
                if current_a == current_b:
                    # Same state: clockwise (depending on encoder wiring)
                    delta = 1
                else:
                    # Different state: counter-clockwise
                    delta = -1
                
                # Apply direction inversion if needed
                if self.invert_direction:
                    delta = -delta
                    
                self.count += delta
                self.last_a = current_a
                
            elif current_b != self.last_b:
                # B changed state
                if current_a == current_b:
                    # Same state: counter-clockwise
                    delta = -1
                else:
                    # Different state: clockwise
                    delta = 1
                
                # Apply direction inversion if needed
                if self.invert_direction:
                    delta = -delta
                    
                self.count += delta
                self.last_b = current_b
            
            # Update timestamp for RPM calculation
            current_time = time.time()
            if current_time - self.last_update_time >= 0.1:  # Update RPM every 100ms
                delta_count = self.count - self.last_count
                # RPM = (delta_count / pulses_per_revolution) * (60 / delta_time)
                # Assuming 20 pulses per revolution (typical encoder)
                # You can adjust this based on your encoder's PPR
                pulses_per_rev = 20
                delta_time = current_time - self.last_update_time
                if delta_time > 0:
                    self.rpm = (delta_count / pulses_per_rev) * (60 / delta_time)
                self.last_count = self.count
                self.last_update_time = current_time
    
    def get_count(self):
        """Get current encoder count"""
        with self.lock:
            return self.count
    
    def get_rpm(self):
        """Get current RPM (approximate)"""
        with self.lock:
            return self.rpm
    
    def reset(self):
        """Reset encoder count to zero"""
        with self.lock:
            self.count = 0
            self.last_count = 0
            self.last_update_time = time.time()
    
    def get_state(self):
        """Get current pin states"""
        return {
            'a': GPIO.input(self.pin_a),
            'b': GPIO.input(self.pin_b),
            'count': self.get_count(),
            'rpm': self.get_rpm()
        }

class DualEncoderSystem:
    """
    Manages two encoders with data logging capabilities
    """
    
    def __init__(self, left_pins=(14, 15), right_pins=(25, 24)):
        """
        Initialize dual encoder system
        
        Args:
            left_pins: Tuple of (pin_a, pin_b) for left encoder
            right_pins: Tuple of (pin_a, pin_b) for right encoder
        """
        self.left_encoder = RotaryEncoder(left_pins[0], left_pins[1], name="Left Encoder")
        self.right_encoder = RotaryEncoder(right_pins[0], right_pins[1], name="Right Encoder")
        self.logging_enabled = False
        self.log_file = None
        self.start_time = time.time()
        
    def enable_logging(self, filename="encoder_log.csv"):
        """Enable data logging to CSV file"""
        try:
            self.log_file = open(filename, 'w')
            self.log_file.write("timestamp,left_count,right_count,left_rpm,right_rpm,delta\n")
            self.logging_enabled = True
            print(f"Logging enabled: {filename}")
        except Exception as e:
            print(f"Failed to open log file: {e}")
            self.logging_enabled = False
    
    def disable_logging(self):
        """Disable data logging"""
        if self.log_file:
            self.log_file.close()
        self.logging_enabled = False
        print("Logging disabled")
    
    def log_data(self):
        """Log current encoder data"""
        if self.logging_enabled and self.log_file:
            timestamp = time.time() - self.start_time
            left_count = self.left_encoder.get_count()
            right_count = self.right_encoder.get_count()
            left_rpm = self.left_encoder.get_rpm()
            right_rpm = self.right_encoder.get_rpm()
            delta = left_count - right_count
            self.log_file.write(f"{timestamp:.3f},{left_count},{right_count},{left_rpm:.2f},{right_rpm:.2f},{delta}\n")
            self.log_file.flush()
    
    def get_delta(self):
        """Get difference between left and right encoder counts"""
        return self.left_encoder.get_count() - self.right_encoder.get_count()
    
    def reset_all(self):
        """Reset both encoders to zero"""
        self.left_encoder.reset()
        self.right_encoder.reset()
        print("Both encoders reset to zero")

def print_help():
    """Print help information"""
    print("\nCommands:")
    print("  r - Reset both encoders")
    print("  l - Reset left encoder only")
    print("  r - Reset right encoder only")
    print("  L - Toggle logging")
    print("  h - Show this help")
    print("  q - Quit")
    print()

def main():
    """Main program"""
    # Encoder pin configuration
    LEFT_A = 14
    LEFT_B = 15
    RIGHT_A = 25
    RIGHT_B = 24
    
    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Create encoder system
    encoders = DualEncoderSystem(
        left_pins=(LEFT_A, LEFT_B),
        right_pins=(RIGHT_A, RIGHT_B)
    )
    
    print("\n" + "="*60)
    print("RASPBERRY PI DUAL ENCODER SYSTEM")
    print("="*60)
    print(f"Left Encoder:  Pins {LEFT_A} (A) and {LEFT_B} (B)")
    print(f"Right Encoder: Pins {RIGHT_A} (A) and {RIGHT_B} (B)")
    print("\nSpin the wheels manually to test")
    print("Press 'h' for commands")
    print("Press Ctrl+C to quit\n")
    
    # Variables for display update
    last_log_time = time.time()
    update_interval = 0.05  # 50ms update rate (20 Hz)
    
    # Command input handling (non-blocking)
    import select
    import sys
    
    try:
        while True:
            start_loop = time.time()
            
            # Get current values
            left_count = encoders.left_encoder.get_count()
            right_count = encoders.right_encoder.get_count()
            left_rpm = encoders.left_encoder.get_rpm()
            right_rpm = encoders.right_encoder.get_rpm()
            delta = encoders.get_delta()
            
            # Create display string
            display = (
                f"\r"
                f"L:{left_count:6d} ({left_rpm:6.1f} RPM)  |  "
                f"R:{right_count:6d} ({right_rpm:6.1f} RPM)  |  "
                f"Δ:{delta:+6d}"
            )
            
            # Add logging indicator
            if encoders.logging_enabled:
                display += "  [LOGGING]"
            
            sys.stdout.write(display)
            sys.stdout.flush()
            
            # Log data at 10 Hz
            current_time = time.time()
            if current_time - last_log_time >= 0.1:  # 10 Hz logging
                encoders.log_data()
                last_log_time = current_time
            
            # Check for keyboard input (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.read(1).lower()
                
                if cmd == 'h':
                    print_help()
                elif cmd == 'r':
                    encoders.reset_all()
                    print("\nBoth encoders reset")
                elif cmd == 'l':
                    encoders.left_encoder.reset()
                    print("\nLeft encoder reset")
                elif cmd == 'e':
                    encoders.right_encoder.reset()
                    print("\nRight encoder reset")
                elif cmd == 'L':
                    if encoders.logging_enabled:
                        encoders.disable_logging()
                    else:
                        timestamp = time.strftime("%Y%m%d_%H%M%S")
                        encoders.enable_logging(f"encoder_log_{timestamp}.csv")
                elif cmd == 'q':
                    raise KeyboardInterrupt
            
            # Maintain update rate
            elapsed = time.time() - start_loop
            if elapsed < update_interval:
                time.sleep(update_interval - elapsed)
                
    except KeyboardInterrupt:
        print("\n\nShutting down encoder system...")
        
    finally:
        # Cleanup
        if encoders.logging_enabled:
            encoders.disable_logging()
        GPIO.cleanup()
        print("GPIO cleaned up")
        print("Program terminated")

def test_mode():
    """
    Test mode - runs for a fixed duration with detailed output
    Useful for debugging and calibration
    """
    LEFT_A = 14
    LEFT_B = 15
    RIGHT_A = 25
    RIGHT_B = 24
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    print("\n" + "="*60)
    print("ENCODER TEST MODE")
    print("="*60)
    print("This test will run for 30 seconds with detailed output")
    print("Spin the wheels to see real-time data")
    print()
    
    # Create encoders
    left_enc = RotaryEncoder(LEFT_A, LEFT_B, name="Left")
    right_enc = RotaryEncoder(RIGHT_A, RIGHT_B, name="Right")
    
    start_time = time.time()
    last_print_time = start_time
    
    try:
        while time.time() - start_time < 30:  # Run for 30 seconds
            current_time = time.time()
            
            if current_time - last_print_time >= 0.1:  # Print every 100ms
                # Get states
                left_state = left_enc.get_state()
                right_state = right_enc.get_state()
                
                # Clear line and print
                sys.stdout.write("\r" + " " * 80 + "\r")
                sys.stdout.write(
                    f"Time: {current_time - start_time:5.1f}s | "
                    f"Left: {left_state['count']:6d} (RPM: {left_state['rpm']:6.1f}) | "
                    f"Right: {right_state['count']:6d} (RPM: {right_state['rpm']:6.1f}) | "
                    f"Delta: {left_state['count'] - right_state['count']:+6d}"
                )
                sys.stdout.flush()
                last_print_time = current_time
            
            time.sleep(0.01)  # Small sleep to prevent CPU overload
            
        print("\n\nTest complete!")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted!")
        
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    # Choose between normal mode and test mode
    # Set to True for test mode, False for normal interactive mode
    TEST_MODE = False
    
    if TEST_MODE:
        test_mode()
    else:
        main()