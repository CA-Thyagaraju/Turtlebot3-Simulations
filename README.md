# Turtlebot3-Simulations

A ROS2 Jazzy and Gazebo Harmonic based robotics simulation workspace focused on localization, sensor fusion, and autonomous mobile robot navigation using TurtleBot3.

This project implements a complete state estimation pipeline from first principles, including sensor modeling, noise injection, and Extended Kalman Filter (EKF) based localization. The objective is to understand how autonomous robots estimate their state in the presence of uncertainty rather than relying solely on existing ROS2 localization packages.

---

## Project Overview

Mobile robots rely on multiple sensors to estimate their position and orientation. Individual sensors are often noisy, biased, or prone to drift over time.

This project simulates a TurtleBot3 robot operating in a 2D environment and fuses information from:

* Wheel Odometry
* Inertial Measurement Unit (IMU)
* Simulated Indoor Localization Measurements

using a custom Extended Kalman Filter to estimate the robot state:

```math
x =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
```

The implementation follows a modular ROS2 architecture with separate sensor, estimation, and visualization nodes.

---

## Features

### Sensor Simulation Layer

Custom ROS2 nodes simulate realistic sensor behavior.

#### Odometry

* Dead-reckoning drift
* Systematic bias
* Accumulated position error

#### IMU

* Gyroscope bias
* Angular velocity noise

#### Indoor Localization Sensor

* Absolute position measurements
* Low-frequency localization noise
* Indoor positioning system simulation

---

### Extended Kalman Filter

The EKF performs state estimation through:

#### Prediction

State propagation using:

* Linear velocity from wheel odometry
* Angular velocity from IMU measurements

#### Correction

Measurement updates using:

* Absolute position measurements
* Heading measurements

from the indoor localization sensor system.

Implemented components include:

* Nonlinear motion model
* Jacobian linearization
* Covariance propagation
* Kalman gain computation
* Innovation calculation
* Angle normalization
* Prediction and correction visualization

---

### Visualization

A dedicated plotting node provides real-time visualization of:

* Planned trajectory
* Noisy odometry trajectory
* Marvelmind measurements
* EKF prediction
* EKF corrected estimate

This enables direct analysis of sensor drift, measurement uncertainty, and filter performance.

---

## System Architecture

```text
Gazebo Simulation
        │
        ▼
Raw Sensor Topics
(/odom, /imu)
        │
        ▼
robot_data
        │
        ├── /odom_data
        ├── /imu_data
        └── /marvelmind_data
        │
        ▼
Extended Kalman Filter
        │
        ▼
Estimated State
[x, y, θ]
        │
        ▼
Trajectory Visualization
```

---

## Package Structure

```text
src/
├── extended_kalman_filter/
│   ├── EKF implementation
│   └── Launch files
│
├── robot_data/
│   ├── Odometry simulation
│   ├── IMU simulation
│   └── Marvelmind simulation
│
├── track_plotter/
│   └── Real-time trajectory visualization
│
└── trajectory/
    ├── Reference trajectory generation
    └── Example trajectories for localization testing
```

### Package Descriptions

| Package                | Description                                                 |
| ---------------------- | ----------------------------------------------------------- |
| extended_kalman_filter | Extended Kalman Filter implementation and launch files      |
| robot_data             | Sensor simulation and noise injection nodes                 |
| track_plotter          | Real-time trajectory visualization and comparison           |
| trajectory             | Reference trajectory generation and example motion profiles |

---

## Available Trajectories

The `trajectory` package contains example reference trajectories that can be used to evaluate localization performance under different motion conditions.

Examples include:

* Straight-line motion
* Circular trajectories
* Custom motion profiles

These trajectories publish reference commands and planned paths that can be compared against:

* Noisy odometry
* Marvelmind measurements
* EKF prediction
* EKF corrected estimate

allowing direct evaluation of localization accuracy and filter performance.

---

## Results

The EKF successfully combines noisy sensor measurements to produce a significantly more stable state estimate than any individual sensor source.

The project demonstrates:

* Odometry drift accumulation
* Measurement uncertainty
* Sensor fusion through EKF
* Improved localization accuracy
* Real-time state estimation

The simulation demonstrates:

* TurtleBot3 operation in Gazebo
* Sensor noise injection
* EKF prediction and correction
* Real-time trajectory tracking

---

## Dependencies

This project relies on the official TurtleBot3 ROS2 packages.

### Software Requirements

* Ubuntu 24.04
* ROS2 Jazzy
* Gazebo Harmonic
* TurtleBot3 Packages
* Python 3
* NumPy
* Matplotlib

### TurtleBot3 Dependencies

Install the official TurtleBot3 packages before running this project.

```bash
sudo apt install ros-jazzy-turtlebot3*
```

Alternatively, clone the official TurtleBot3 repositories into your ROS2 workspace.

---

## Build Instructions

```bash
mkdir -p ~/turtlebot3_ws/src

cd ~/turtlebot3_ws/src

git clone https://github.com/CA-Thyagaraju/Turtlebot3-Simulations.git

cd ..

rosdep install --from-paths src --ignore-src -r -y

colcon build

source install/setup.bash
```

---

## Running the Simulation

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch extended_kalman_filter ekf.launch.py
```

This launch file automatically:

* Starts Gazebo
* Spawns the TurtleBot3 model
* Launches sensor simulation nodes
* Launches the EKF localization node
* Launches the visualization node

---

## Motivation

The goal of this project was to develop a practical understanding of:

* State Estimation
* Sensor Fusion
* Robot Localization
* Mobile Robot Kinematics
* ROS2 System Architecture
* Autonomous Navigation Fundamentals

Rather than treating localization frameworks as black boxes, the focus was placed on understanding the mathematical and engineering principles behind real robotic state estimation systems.

---

## Current Status

Implemented:

- TurtleBot3 simulation in Gazebo
- Sensor noise modeling
- Odometry, IMU and localization sensor abstraction
- Extended Kalman Filter localization
- Real-time trajectory visualization

Currently exploring:

- Closed-loop trajectory tracking
- Pure Pursuit and Stanley controllers
- Hardware deployment on TurtleBot3

## Future Work

Planned extensions include:

* Pure Pursuit Controller
* Stanley Controller
* Closed-loop trajectory tracking
* Hardware deployment on TurtleBot3
* Sensor calibration and system identification
* SLAM experimentation
* Integration with the ROS2 Nav2 stack

---

## Author

### Chilkunda Achutha Thyagaraju

Electrical Engineering Undergraduate
National Institute of Technology Rourkela
+91 82961 71669 achuthathyagaraju@gmail.com


**Areas of Interest**

* Robotics
* Autonomous Systems
* Control Systems
* Sensor Fusion
* State Estimation
* Embedded Systems
* Mobile Robot Navigation
