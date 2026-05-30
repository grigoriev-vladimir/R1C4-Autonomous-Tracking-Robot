# R1C4 - Autonomous Target Tracking Robot (Scale 3/5)

R1C4 is a 3/5 scale R2-D2 replica equipped with computer vision to autonomously detect, track, and follow its "master" using a motorized turret system. This project combines computer vision, advanced mechatronics, and real-time control loops.

---

## 🎥 Demo
<!-- TIP: Once your repo is ready, upload a GIF or a short video of R1C4 tracking you here -->
![R1C4 in Action](media/demo.gif)
*Click here to watch the full video demonstration on [YouTube/LinkedIn/etc.]*

---

## ✨ Key Features
* **Person Detection & Tracking:** Real-time human tracking using Computer Vision.
* **Active Turret Control:** Motorized dome/turret that smoothly pans to keep the target centered in the camera's field of view.
* **Custom Mechatronics:** Integrated power management, motor drivers, and 3/5 scale mechanical integration.

---

## 🛠️ Hardware Architecture
The robot architecture bridges high-level processing with low-level hardware actuation:

| Component Type | Model / Specification | Function |
| :--- | :--- | :--- |
| **Main Compute** | [e.g., Raspberry Pi 4 / Laptop] | Runs the computer vision model and processes video feed |
| **Microcontroller** | [e.g., Arduino Uno / STM32] | Handles low-level motor control and sensor reading |
| **Camera** | [e.g., Raspberry Pi Cam V2 / USB Webcam] | Captures real-time video stream for tracking |
| **Actuators** | [e.g., Nema 17 Stepper / Servo MG996R] | Drives the dome/turret rotation |
| **Motor Driver** | [e.g., L298N / TMC2209] | Interfaces microcontroller commands with the motors |
| **Power Supply** | [e.g., LiPo Battery 3S 11.1V] | Powers the computing unit and high-torque motors |

---

## 🧠 Software & Control Logic

### 1. Vision Pipeline
The high-level compute unit processes the video stream frame-by-frame:
* **Framework:** OpenCV [and state any model used like YOLO, Haar Cascades, or Mediapipe].
* **Logic:** The script detects the user, extracts the bounding box center coordinates $(X_{target}, Y_{target})$, and calculates the pixel offset relative to the camera's center $(X_{center}, Y_{center})$.

### 2. Control Loop (Feedback System)
The tracking error is sent to the microcontroller to drive the turret motors:
* **Algorithm:** [Mention if you used a PID controller or a simple proportional loop].
* The system dynamically adjusts the motor speed and direction to minimize the error and maintain smooth, fluid tracking without jerky movements.

---

## 📁 Repository Structure
```text
├── src/
│   ├── vision/       # Python scripts for computer vision and target tracking
│   └── control/      # C++/Arduino firmware for motor control loops
├── hardware/
│   ├── electronics/  # Wiring diagrams and schematics
│   └── 3d-models/    # STL files for the custom turret mechanism
└── media/            # Images and GIFs for documentation
