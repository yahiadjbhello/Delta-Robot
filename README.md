# Delta Robot Control with PID using ESP32 and Real-Time GUI

## Overview

This project presents the design and implementation of a **Delta Robot control system** using an **ESP32 microcontroller** and a **PID control algorithm**. The system allows real-time control of the robot motion while visualizing and comparing the **desired trajectory** and the **measured trajectory** through a graphical user interface.

The main objective of this project is to develop a complete control architecture for a Delta Robot, including embedded control, trajectory tracking, sensor feedback, and real-time monitoring using a GUI.

---

## Project Description

The Delta Robot is a parallel robotic manipulator commonly used in high-speed pick-and-place applications. In this project, the robot is controlled using stepper motors driven by an ESP32. The position feedback is measured using magnetic rotary encoders, allowing the control system to compare the desired position with the actual position.

A PID controller is implemented to improve the accuracy, stability, and tracking performance of the robot. The system also includes a GUI developed in Python to monitor the robot behavior in real time.

The GUI allows the user to:

- Send trajectory commands to the robot
- Visualize the desired trajectory
- Visualize the measured trajectory
- Compare tracking performance in real time
- Monitor the robot response during motion

---

## Main Features

- Real-time Delta Robot control using ESP32
- PID-based position control
- Stepper motor control
- Rotary encoder feedback using AS5600 sensors
- Real-time trajectory tracking
- Desired vs measured trajectory visualization
- Python-based graphical user interface
- Serial communication between ESP32 and PC
- Mechanical design files included
- Motor and encoder documentation included

---

## System Architecture

The system is composed of four main parts:

1. **Mechanical Structure**
   - Delta Robot mechanical parts
   - Fixed base
   - Moving platform
   - Upper arms
   - Lower arms
   - Spherical joints
   - End-effector

2. **Embedded Control**
   - ESP32 microcontroller
   - Stepper motor control
   - PID control algorithm
   - Encoder data acquisition

3. **Sensing System**
   - AS5600 magnetic rotary encoders
   - Position feedback measurement
   - Real-time angle reading

4. **Graphical User Interface**
   - Python GUI
   - Real-time trajectory plotting
   - Desired and measured trajectory comparison
   - Communication with ESP32 through serial port

---

## Repository Structure

```text
Delta_robot_project/
│
├── Control code with PID in ESP32/
│   └── ESP32withGUI.ino
│
├── Delta_Robot/
│   │
│   ├── Delta robot/
│   │   ├── end-effector_default_sldprt.stl
│   │   ├── fixed-base_default_sldprt.stl
│   │   ├── h1_default_sldprt.stl
│   │   ├── h2_default_sldprt.stl
│   │   ├── lower-arme_default_sldprt.stl
│   │   ├── motor_default_sldprt.stl
│   │   ├── mouving-platform_default_sldprt.stl
│   │   ├── spherical-joint_default_sldprt.stl
│   │   └── upper-arme_default_sldprt.stl
│   │
│   ├── Encoder/
│   │   ├── AS5600-PIC-2.jpg
│   │   └── Magnetic Rotary Position Sensor AS5600 Datasheet.pdf
│   │
│   └── Motors/
│       ├── Stepper Motor .jpg
│       └── Stepper Motor Specifications.pdf
│
├── GUI/
│   └── gui.py
│
├── Kinematics equations/
│   └── kinematics inverse in frensh.pdf
│
└── README.md
