# Control-Of-Autonomous-System
# Modelling and Nonlinear Control of Quadcopter

This project involves the modeling and nonlinear control of a quadcopter, a four-rotor aerial vehicle. The report explores its dynamics, control systems, and simulation results for stabilization and trajectory tracking. The focus is on implementing a Proportional-Derivative (PD) controller to achieve desired behaviors.

## Project Details

- **Course:** IE 415 - Control of Autonomous Systems  
- **Author:** Thakkar Dishank R (Student ID: 202201518)  
- **Supervisor:** Prof. Sujay D Kadam  

---

## Overview

A quadcopter is an underactuated nonlinear system capable of six degrees of freedom. This project covers:

- **System Modeling:**  
  - Quadcopter dynamics, including position and orientation in inertial and body frames.
  - Mathematical modeling with assumptions for control purposes.
  - Implementation of Euler angles and rotational matrices for frame transformations.  

- **Control Implementation:**  
  - Use of a PD controller to stabilize the quadcopter.  
  - Torque and thrust equations derived for roll, pitch, and yaw.  

- **Simulations:**  
  - Stabilization tests with and without PD control.  
  - Trajectory tracking for circular and spiral paths.  

---

## Features

1. **Nonlinear Dynamics Analysis:**  
   In-depth understanding of the quadcopter’s behavior under various forces and torques.  

2. **PD Controller Design:**  
   A robust control strategy to ensure stability and accurate trajectory tracking.  

3. **Simulation Results:**  
   Insights into the quadcopter's stabilization and trajectory following using MATLAB simulations.

---


## Getting Started

### Prerequisites

- MATLAB/Simulink (Recommended for running the simulations)
- Basic understanding of control systems and dynamics

---

## Overview

A quadcopter is an underactuated nonlinear system capable of six degrees of freedom. This project covers:

- **System Modeling:**  
  - Quadcopter dynamics, including position and orientation in inertial and body frames.
  - Use of Euler angles and rotational matrices for frame transformations.  

- **Control Implementation:**  
  - Proportional-Derivative (PD) controller for stabilization and trajectory tracking.
  - Torque and thrust equations derived for roll, pitch, and yaw.
  - Feedback Linearization controller for advanced control testing.

- **Simulations:**  
  - Stabilization with and without control.
  - Trajectory tracking for circular and spiral paths.

---

## Key Simulation Features

### Control Modes
The simulation supports the following modes:
1. **Stabilization:** Maintain hover position with PD control.  
2. **Trajectory Tracking:** Follow a predefined path (e.g., circular or spiral trajectories).  
3. **Feedback Linearization:** Advanced control to directly linearize system dynamics.

### Adjustable Parameters
- **Trajectory Type:** Circular or spiral.  
- **Control Gains:** Tunable parameters for PD control (`Kp`, `Kd`) for precise performance.  
- **Simulation Time:** Configurable based on desired duration and trajectory complexity.

### Notations
- **State Variables:**
  - **Position:** \([x, y, z]\)
  - **Linear Velocities:** \([u, v, w]\) (body frame) and \([\dot{x}, \dot{y}, \dot{z}]\) (inertial frame)
  - **Angular Velocities:** \([p, q, r]\) (body frame) and \([\dot{\phi}, \dot{\theta}, \dot{\psi}]\) (inertial frame)  

---

## Running the Simulation

1. Open the MATLAB script and configure the desired parameters:
   - **Control Mode:** Select `1` for stabilization, `3` for trajectory tracking, or `4` for feedback linearization.
   - **Trajectory Type:** Use `param.traj = 1` for circular or `param.traj = 2` for spiral.
   - Adjust PD gains or trajectory properties as required.

2. Run the script to simulate quadcopter behavior.

3. View results in the generated plots:
   - Control inputs for each rotor.
   - Position and angle evolution over time.
   - 3D trajectory visualization comparing actual and desired paths.

---

## References

1. M. Etemadi, "Mathematical dynamics, kinematics modeling, and PID equation controller of quadcopter," IJOR, 2017.  
2. S. Bouabdallah, "Design and control of quadrotors," EPFL, 2007.  
3. F. Sabatino, "Quadrotor control: modeling, nonlinear control design, and simulation," 2015.  
4. Additional references are included in the project report.

---

## Author

Dishank R Thakkar  
IE 415, Control of Autonomous Systems  
202201518  

For questions or feedback, feel free to reach out.
