# Precision Optical Alignment System

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

Automated optical alignment system combining **fast steering** (piezo + quadrant detector) and **slow steering** (stepper/motorized stages) with Python scripts and GUI.

---

## Table of Contents

- [Overview](#overview)  
- [Features](#features)  
- [Hardware Requirements](#hardware-requirements)  
- [Software Requirements](#software-requirements)  
- [Project Structure](#project-structure)  
- [Usage](#usage)  
- [Calibration](#calibration)  
- [Example Workflow](#example-workflow)  
- [Contributing](#contributing)  
- [License](#license)  

---

## Overview

This project provides Python tools to automate optical beam alignment:

- **Fast Steering**: Real-time correction using KPA101 piezo + quadrant detector.  
- **Slow Steering**: Coarse alignment using stepper/motorized stages with PID control.  
- **Manual Control**: CLI and GUI options.  
- **USB Device Auto-Detection**: Maps hardware to `/dev/ttyUSB*` using product names or serials.

The system reduces manual intervention and stabilizes optical beams over time.

---

## Features

- Fast feedback loop for optical beam stabilization.  
- Automated slow-axis alignment via PID.  
- Manual CLI control and GUI interface.  
- Automatic device detection.  
- Modular design for easy hardware replacement or expansion.

---

## Hardware Requirements

| Component | Purpose |
|-----------|---------|
| Thorlabs KPA101 | Piezo + Quadrant Detector for fast steering |
| Thorlabs KDC101 | Motorized slow steering |
| Pololu Tic USB Stepper | Stepper motor control for slow axes |
| Raspberry Pi / Desktop | Host computer running Python scripts |

---

## Software Requirements

- Python 3.x  
- Python libraries:
  - `numpy`
  - `pylablib` (Thorlabs hardware interface)
  - `ticlib` (Pololu Tic stepper interface)
  - `pyserial` (USB-serial communication)
  - GUI library (Tkinter or PyQt, depending on `gui_app.py`)

---

## Project Structure

| File | Description |
|------|-------------|
| `fast_steering.py` | Fast closed-loop feedback control using piezo + detector |
| `slow_steering.py` | PID-controlled slow steering via stepper/motorized stages |
| `optical_aligner.py` | Combines fast and slow steering routines for complete alignment |
| `manual_steering.py` | CLI for manual axis adjustments |
| `pid_controller.py` | Generic PID control module |
| `gui_app.py` | GUI for real-time control and monitoring |
| `usbfinder.py` | Maps USB devices to `/dev/ttyUSB*` by product name/serial |
| `instruction.txt` | Setup and usage instructions |
| `config/` | Configuration files for device IDs, calibration, and default settings |

---
