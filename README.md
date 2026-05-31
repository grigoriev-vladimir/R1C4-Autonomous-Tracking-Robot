# R1C4 - Autonomous Target Tracking Robot (Scale 3/5)

R1C4 is a 3/5 scale R2-D2 replica, co-developed as a duo with Emma Da Mota. It is equipped with computer vision to autonomously detect, track, and follow its "master" using a motorized turret system. This project combines computer vision, advanced mechatronics, and real-time control loops.

---

## 🎥 Demo
<!-- TIP: Once your repo is ready, upload a GIF or a short video of R1C4 tracking you here -->


https://github.com/user-attachments/assets/d546fa72-e5a3-4e6c-b071-dcd2d76cd6c8




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

### 1. Vision & Recognition Pipeline
The high-level compute unit processes the webcam video feed frame-by-frame using a dual-stage approach:
* **Face Recognition (LBPH):** The system uses the **LBPH (Local Binary Patterns Histograms)** recognizer from OpenCV. It identifies the specific face of the "Master" based on a pre-trained dataset of face samples.
* **Target Tracking (CSRT):** Once the master is recognized, the system initializes an OpenCV **CSRT Tracker** on the target. This ensures robust, real-time tracking even with fast movements or partial occlusions.
* **Logic:** The Python script extracts the tracking bounding box center coordinates and calculates the horizontal pixel error relative to the camera's center view.

### 2. Communication & Control Loop
The tracking error is sent in real-time to the microcontroller to drive the robot's actuators:
* **Data Transmission:** The calculated pixel error is transmitted from the NVIDIA Jetson Nano to the **ESP32** via a **Serial (UART) link**.
* **Turret Actuation:** The ESP32 processes this error to command the **TMC2209** driver. It dynamically adjusts the stepper motor's speed and direction to smoothly and silently rotate the dome, keeping the master centered in the camera's field of view.
* **Mobility:** In parallel, the ESP32 handles the robot's displacement by driving the main DC motors through the **MDD10A** smart driver based on the system's operational state.
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
