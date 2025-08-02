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
<img width="1265" height="545" alt="{A4C5181D-CBCE-4E28-9B8A-BD87CC7754A1}" src="https://github.com/user-attachments/assets/6acd131e-79f8-4df6-b624-223fbfc06979" />
<br>
Where are applied some constraint on δ(steering angle), position X and Y.

