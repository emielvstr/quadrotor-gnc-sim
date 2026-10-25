# 3D Quadrotor Guidance, Navigation & Control (GNC) Simulator

![Python](https://img.shields.io/badge/Python-3.9+-blue.svg)
![CasADi](https://img.shields.io/badge/Optimization-CasADi-orange)
![Control](https://img.shields.io/badge/Control-NMPC%20%7C%20Cascaded-brightgreen)
![Filter](https://img.shields.io/badge/State%20Estimation-ESKF-purple)

A high-fidelity, from-scratch 3D Quadrotor simulator built in Python. This project serves as a comprehensive playground for testing advanced Guidance, Navigation, and Control algorithms, heavily relying on rigid-body physics, quaternion math, and mathematical optimization.

![Quadrotor Simulation](Results/3d_vlucht.gif)

## Key Features

*   **Rigid-Body Dynamics:** 6-DOF Newton-Euler equations of motion using **Hamilton Quaternions** to prevent Gimbal Lock, integrated via an RK4 (Runge-Kutta 4th order) solver.
*   **Sensor Fusion (ESKF):** An Error-State Kalman Filter combining noisy, high-frequency IMU data with low-frequency GPS data to accurately estimate the true state, utilizing local SO(3) error mapping.
*   **Model Predictive Control (NMPC):** A Non-Linear MPC implementation using the **CasADi** framework and IPOPT solver, allowing for real-time optimal trajectory tracking while respecting actuator limits.
*   **Cascaded Baseline Controller:** A robust classic cascaded architecture (Outer Loop: Position to Thrust/Attitude, Inner Loop: Attitude to Motor Torques).
*   **Differential Flatness Trajectories:** Quintic polynomial trajectory generation ensuring continuous position, velocity, and acceleration profiles.
*   **3D Telemetry Visualization:** A custom Matplotlib Dark-Mode rendering engine including simulated propeller discs, ground shadow projection, and a live HUD.

## Project Architecture

The simulator is built in a highly modular fashion:

| Module | Description |
| :--- | :--- |
| `config.py` | Physical drone parameters (mass, inertia matrix $J$, aerodynamic coefficients). |
| `drone_dynamics.py` | The physics engine. Converts motor speeds to forces/torques and updates the 13-variable state vector. |
| `trajectory.py` | Generates smooth references ($x, y, z, \psi$) using Quintic Polynomials. |
| `controller.py` | The Cascaded PID Controller (Geometric tracking). |
| `mpc_controller.py` | The Non-Linear Model Predictive Controller using CasADi. |
| `sensors.py` | Generates Gaussian noise for virtual IMU and GPS sensors. |
| `eskf.py` | The Error-State Kalman Filter for robust state estimation. |
| `visualize.py` | 3D animation and GIF export engine. |

