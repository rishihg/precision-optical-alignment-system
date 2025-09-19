### Optical Beam Alignment Control System
This project contains a suite of Python tools for controlling an automated optical alignment system. It provides separate interfaces for slow, high-precision alignment using stepper motors and fast, real-time beam stabilization using a KPA101 piezo controller and a quadrant detector.

### Features
**Fast Steering Mode**: Utilizes the hardware's internal closed-loop feedback for real-time beam stabilization with interactive mode and settings control.

**Slow Steering Mode**: A software-based PID control loop for automated, high-precision alignment using stepper motors. Includes an automated calibration routine.

**Manual Control**: Direct command-line and GUI interfaces for manual control of stepper motors for coarse alignment and setup.

**Robust Hardware Handling**: Automatically discovers devices by serial number, preventing connection issues from changing USB ports.

### Hardware & Software
This system is designed around a specific set of hardware components and Python libraries.

**Hardware Used**
Fast Actuator / Detector: Thorlabs KPA101 K-Cube Piezo Driver & Quadrant Detector. This single unit provides both the high-speed piezo mirror control and the beam position sensing required for the fast feedback loop.

Slow X-Axis Actuator: A standard stepper motor controlled by a Pololu Tic USB Stepper Motor Controller.

Slow Y-Axis Actuator: A goniometer or linear stage driven by a Thorlabs Kinesis Motor Controller (e.g., KDC101 Servoc controller).

Host Computer: A Raspberry Pi 5 or a standard desktop computer running Python.

**Software & Libraries**
Python 3: The core programming language.

pylablib: For communication with Thorlabs Kinesis devices (KPA101 and Kinesis Motor).

ticlib: For communication with the Pololu Tic stepper motor controller.

numpy: For numerical operations, particularly matrix math in the calibration routine.