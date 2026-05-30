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
| **Main Compute** | NVIDIA Jetson Nano | Runs the AI vision pipeline and processes the webcam feed |
| **Microcontroller** | ESP32 | Manages low-level motor control and timing |
| **Camera** |Logitech Webcam | Captures real-time video stream for tracking |
| **Turret Actuator** | Stepper Motor + TMC2209 | Drives the dome rotation smoothly and silently |
| **Wheel Actuator** | DC Motors + MDD10A SmartDrive | Controls the robot's main mobility and platform movement |
| **Power Supply** | External Power Bank / Battery | Provides independent power to the compute unit and motors |

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
