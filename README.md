# ADDER
Code used to control **A** **D**ouble-spiral **D**ual-motor actuat**E**d snake **R**obot (**ADDER**). This snake robot uses a double-spiral actuator capable of both rotational and translational motion. These actuators are used as joints, connected orthogonally in series, enabling the robot to move within a 3D workspace. Due to the actuator's 2-DOF motion and the arrangement of the joints, ADDER can perform all four fundamental snake gaits: serpentine, sidewinding, rectilinear, and concertina.

This repository contains all the code used to control ADDER and collect sensory data from the robot for post-processing and analysis.

## Features

- Dynamixel controller for controlling multiple dynamixel motors at once.
- Snake Controller which could generate all biological snake gaits given its parameters.
- Controls the double-spiral structure with the given angle or position.
- Records sensor data for post-processing

## Hardware Requirements

- ADDER
- Raspberry Pi 3 Model B (or Raspberry Pi 3 Model B+)
- Dynamixel servo motors
- MPU6050 IMU sensor
- INA219 current/voltage sensor
- Power supply for the robot

## Software Requirements

### Raspberry Pi
- Raspberry Pi OS with the TurtleBot image
- Python 3.8+

### PC (Host Computer)

One of the following:

- Ubuntu 20.04 LTS (recommended)
- Windows 11 with Ubuntu via WSL2
- macOS with SSH support

The host computer must be able to:
- Connect to the Raspberry Pi over SSH.
- Be on the same network as the robot.
- Run the required Python scripts and dependencies.

## Installation

Clone the repository:

```bash
git clone https://github.com/VISTEC-IST-ROBOTICS-PUBLIC/ADDER.git
cd ADDER
```

Clone dynamixel-sdk
```bash
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

Install dependencies
```bash
pip install numpy
pip install pandas
pip install smbus2
```


## Usage

Run the controller:

```bash
python main.py
```
