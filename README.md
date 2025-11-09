# RoboSort: Automated Sorting Robot

## Project Overview
RoboSort is an automated sorting robot designed to efficiently separate and organize objects using a robotic arm and conveyor system. The project integrates both Arduino and Raspberry Pi platforms, combining mechanical, electrical, and software components for a robust, scalable solution suitable for educational, prototyping, and light industrial applications.

## Features
- Five degrees of freedom (5-DOF) robotic arm driven by a 16-channel PWM servo driver
- Modular design with clear separation between control (Arduino) and high-level logic (Raspberry Pi)
- Serial command interface for manual testing and calibration
- Expandable architecture for additional sensors or actuators
- Comprehensive documentation and model images for easy assembly and understanding

## Hardware Specifications

### Main Body
- Height: 4 inches
- Length: 35 inches
- Width: 20 inches

### Trash Bin
- Height: 20 inches
- Radius: 10 inches (Diameter: 20 inches)

### Robotic Arm
- 5-DOF articulated arm
- Controlled by 5 standard servos (SG90/MG996R or similar)
- Driven by Adafruit 16-channel PWM Servo Driver (PCA9685)

### Electronics
- Arduino-compatible microcontroller (Uno, Mega, etc.)
- Raspberry Pi (for high-level control, vision, or networking)
- Power supply for servos and logic

## Software Structure

### Arduino
- `RoboSort.ino`: Main firmware, handles serial commands and servo control
- `SERVO_CONFIG.h/.cpp`: Encapsulated servo configuration and control logic

#### Serial Commands
- `TEST`: Runs a test sequence on all servos
- `S<servo> <angle>`: Sets a specific servo (0-4) to a given angle (0-180)

### Raspberry Pi
- `RoboSort.py`: Placeholder for high-level logic, integration with sensors, or advanced features

## Model Images

### Front View
![RoboSort Front View](model/robosort-front.jpg)
The front view shows the main interface and access points of the RoboSort system, including the entry slot for sorting and the control panel.

### Side View
![RoboSort Side View](model/robosort-side.jpg)
The side view highlights the internal mechanism layout, conveyor system, and the relative position of the trash bin to the sorting area.

### Main/Top View
![RoboSort Main View](model/robosort-main.jpg)
The main/top view provides a comprehensive look at the overall structure, showing the arrangement of components and the spatial relationship between the body and the trash bin.

## Getting Started

### Prerequisites
- Arduino IDE (for firmware upload)
- Adafruit PWM Servo Driver library
- Python 3.x (for Raspberry Pi logic)

### Setup Instructions
1. Assemble the mechanical structure as shown in the model images.
2. Connect the servos to the PWM driver and wire the electronics as per the hardware specifications.
3. Upload the Arduino firmware (`RoboSort.ino`) to your microcontroller.
4. Use the serial monitor to test and calibrate the servos using the provided commands.
5. (Optional) Develop and run high-level logic on the Raspberry Pi using `RoboSort.py`.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Contact
For questions, suggestions, or contributions, please open an issue or contact the repository owner.