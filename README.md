# Linear MPC for Steering Control
UNIFI Bachelor's degree Thesis Work: MPC for Formula Student Autonomous System

## Introduction
This project implements a Model Predictive Control (MPC) algorithm for the steering control of an autonomous Formula Student Driverless vehicle. It uses a linearized bicycle model (LTV) and solves a quadratic programming (QP) problem at each time step to track a reference trajectory while respecting physical constraints.
## Formulation
This MPC try to follow a predefined reference path using a bycicle linear time-varying model, with linear Pacejka Magic Formula, with the following state equations:
<br>
<img width="587" height="240" alt="{FC57B790-AE6C-419E-93FD-FF908BCBAC55}" src="https://github.com/user-attachments/assets/d530a143-2f2b-4f75-8411-4b9079c3ce39" />
<br>
in which, at every time step, the values of Vlon, θ and Ki(cornering stiffness) are updated. The values of Vlon and θ are respectively read from wheels sensors, and SLAM algorithm, instead the Ki values are read from a lookup table hat correlates vertical load on the wheel to cornering stiffness.<br>
The state of the problem are given as follows:
<br>
<img width="138" height="244" alt="{473A1425-9143-4F33-ABE1-4BE50D7982B1}" src="https://github.com/user-attachments/assets/c6f4b0ed-a0e7-418d-b091-fd7e4dde365e" />
<br>
<br>
<br>
The cost function, to formulate the linear optimization problem, is as follow:
<br>
<img width="725" height="308" alt="{185A0332-64B5-4987-911D-9C9ADD9F09C6}" src="https://github.com/user-attachments/assets/206c29c9-fce2-4e33-b89e-f417b45298de" />
<br>
Where are applied some constraint on δ(steering angle), position X and Y.

## Dependencies
This MPC works with the following libraries:
- OSQP (linear optimization problem solver)
- Eigen (for linear algebra data stuctures)
- Eigen-osqp (wrapper for eigen and osqp functions)

## Example on how it works
Here is an example on how it works on Formula Student Driverless Simulator "https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0" (sorry for the bad quality)
<br>
<br>
