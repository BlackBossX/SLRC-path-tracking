# SLRC26 Autonomous Line-Following and Object-Grabbing Robot - Elimination Round

This repository contains the complete software stack for an autonomous Raspberry Pi-based robot competing in the **SLRC26 (Service League Robot Challenge) Elimination Round**. The robot is designed to autonomously navigate a precisely marked track, detect and locate payload objects placed in designated zones, retrieve them using a robotic manipulator arm, and dispense them at a collection point.

## 🎯 Mission Objective

The robot must:
1. Start at a designated point and locate the initial track line
2. Follow a white line through a course containing multiple corridors/T-junctions
3. Detect when it encounters a side corridor (via ultrasonic distance drop)
4. Navigate perpendicular into the corridor to locate a red payload
5. Autonomously grab the payload using a 3-DOF robotic arm
6. Return the payload to a designated drop zone
7. Continue indefinitely, repeating the cycle for multiple payloads

## ✨ Key Accomplishments

- **Robust Encoder Integration**: Implemented a custom `RotaryEncoder` class with hardware-threaded quadrature decoding and software debouncing to eliminate noise and provide accurate distance/angle odometry tracking.
- **Smooth PID Line Following**: Developed a Proportional-Integral-Derivative (PID) control system that keeps the robot centered on the white line with minimal oscillation, even at turns and T-junctions.
- **Multi-threaded Sensor Fusion**: Created an asynchronous ultrasonic polling thread with median filtering to detect payloads without blocking the main vision loop.
- **Intelligent Vision System**: Integrated HSV color masking to precisely locate red payloads and T-junction detection via bounding box width analysis.
- **Precision Arm Control**: Implemented smooth servo actuation sequences (Elbow, Wrist, Gripper) to safely grab and place objects, with dynamic PWM release to prevent servo jitter when idle.
- **Active Braking & Momentum Control**: Added reverse-thrust braking to immediately stop chassis drift when centering onto payloads before grabbing.
- **Stepper Motor Dispensing**: Integrated a 4-coil stepper motor controller (ULN2003) to cycle the payload chamber mechanism 100 steps per drop.

## 🚀 Core Features

- **Real-time Computer Vision**: Uses Picamera2 and OpenCV to detect white track lines and red payloads
- **Closed-Loop Odometry**: Quadrature encoders provide precise distance and rotation feedback for perfect navigation
- **Dynamic Obstacle Mapping**: Ultrasonic sensors continuously map the environment and trigger payload search sequences
- **Autonomous Arm Manipulation**: Smooth, choreographed servo movements for reliable grabbing and placing
- **Stateful Mission Control**: Structured 6-phase state machine ensures predictable, repeatable behavior

## 🧬 Mission Phasing Logic

The robot operates through a structured State Machine with 6 distinct phases:

**Phase 0 (Initialization & Track Seek)**
- Stepper motor reverses 100 steps to establish a safe zero position
- Robot creeps forward slowly (Speed: 18) scanning for the horizontal white line
- Upon detection, executes a -90° pivot to enter the main track

**Phase 1 (Baseline Calibration)**
- Establishes expected wall distances at `55.0cm` for left/right corridors
- Prevents false positive triggers from wall noise

**Phase 2 (Line Tracking & Payload Detection)**
- Continuous PID adjustment of motor speeds to keep robot centered on white line
- Asynchronous ultrasonic polling detects sudden distance drops (payload zones)
- Logs which side the payload was detected (LEFT or RIGHT)

**Phase 3 (Payload Alignment)**
- Executes a 90° pivot toward the detected corridor
- Backs up 3cm to allow proper camera focal distance for payload detection

**Phase 4 (Red Payload Capture)**
- HSV color filtering locates the red object centroid
- Robot self-aligns and creeps forward until 17cm proximity is reached
- Immediate active braking prevents momentum-induced misalignment
- Arm smoothly executes grab sequence, holds object, then places it
- Stepper motor cycles 100 steps clockwise to dispense payload
- Servos release PWM power to eliminate jitter while idle

**Phase 5 (Course Recovery)**
- Backs up 2cm from the drop zone
- Inversely pivots 100° to realign with the main track (avoids full 180° turn)
- Seamlessly resumes Phase 2 tracking indefinitely

## ⚙️ Software Requirements

- Python 3.x
- `RPi.GPIO`
- `opencv-python` (`cv2`)
- `picamera2`
- `numpy`

## 🏃 Running the Robot

Ensure your Raspberry Pi camera is enabled, then execute:
```bash
sudo python3 integrated_line_and_grab.py
```

## 📂 Key Files

- **`integrated_line_and_grab.py`** - Main production script; implements all 6 phases, vision pipeline, arm control, and mission logic
- **`test_encoders_only.py`** - Encoder calibration and validation utility
- **`stepper_test.py`** - Isolated stepper motor testing module

## 🎓 Technical Insights

- Encoder ticks-per-cm calibration: ~5.5 ticks/cm
- Servo-free holdover prevents heating and jitter during idle periods
- Median filtering on ultrasonic readings reduces spike sensitivity
- Active reverse braking (500ms pulse at -15 PWM) counteracts friction skidding
- T-junction detection uses bounding box width >80% screen threshold

---

## 📋 Implementation Status

**Completed:**
- ✅ Line following with PID control
- ✅ T-junction detection and navigation
- ✅ Ultrasonic-based payload detection and corridor identification
- ✅ Red box detection and centroid centering using HSV color masking
- ✅ Autonomous grabbing and securing the red payload on the robot
- ✅ Navigation through the complete course path

**Not Implemented (Time Constraint):**
- ❌ Payload insertion/dispensing mechanism into the collection tunnel
- ❌ Stepper motor sequence for payload chamber cycling (code present but not field-tested)

*This is the final competition elimination round code. The core autonomous navigation, detection, and payload capture functionalities have been tested and verified for robust operation. The robot successfully completes the main challenge of finding and securing payloads while navigating the track.*
