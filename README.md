# SLRC26 Line Follower Robot

This project is a vision-based line-following robot built with a Raspberry Pi. It uses the Raspberry Pi Camera and OpenCV to detect a black line on the ground, calculate the error from the center of the camera view, and compute the simulated RPM needed for the left and right motors to keep the robot on track.

## Features

- **Computer Vision Line Detection:** Uses OpenCV to threshold a Region of Interest (ROI) at the bottom half of the frame to isolate a black line.
- **Proportional Steering (P-Control):** Computes an `error` value based on the horizontal offset of the line's centroid and adjusts motor RPMs accordingly.
    - If the line drifts right -> Left motor speeds up, Right motor slows down.
    - If the line drifts left -> Right motor speeds up, Left motor slows down.
- **Visual HUD:** Displays bounding boxes around the line, the center deviation, the current pixel error, and the calculated Left/Right RPM.

## Hardware Requirements (Planned)

- **Raspberry Pi** (e.g., Pi 4 or Pi 5)
- **Raspberry Pi Camera Module** (supported by libcamera/picamera2)
- Differential Drive Chassis (2 DC motors)
- Motor Driver (e.g., L298N, TB6612FNG)

## Software Dependencies

You will need the following Python libraries installed:

```bash
pip install opencv-python numpy
```
*Note: `picamera2` usually comes pre-installed on modern Raspberry Pi OS releases based on Debian Bullseye/Bookworm.*

## Usage

1. **Physical Setup:** Ensure your Raspberry Pi Camera is connected and enabled.
2. **Lighting:** The threshold for the black line is currently set to `60` in the `cv2.threshold()` call. You may need to adjust this depending on the lighting in your room and the exact shade of the line/floor.
3. **Run the Tracker:**
   ```bash
   python main.py
   ```
4. **Exit:** Press the `q` key with the video window selected to stop the script and close the camera gracefully.

## Next Steps / Future Work

- **Integrate Motor Control:** Replace/supplement the simulated RPM printouts with actual GPIO PWM outputs using a library like `gpiozero` or `RPi.GPIO`.
- **Tune PID Controller:** Currently, only Proportional (`KP`) control is implemented. You may want to add Derivative (`KD`) and Integral (`KI`) terms for smoother movement.