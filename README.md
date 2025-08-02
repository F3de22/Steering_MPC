# Linear MPC for Steering Control
UNIFI Bachelor's degree Thesis Work: MPC for Formula Student Autonomous System

## Introduction
This project implements a Model Predictive Control (MPC) algorithm for the steering control of an autonomous Formula Student Driverless vehicle. It uses a linearized bicycle model (LTV) and solves a quadratic programming (QP) problem at each time step to track a reference trajectory while respecting physical constraints.
## Formulation
This MPC try to follow a predefined reference path using a bycicle linear time-varying model, with the following state equations:
<br>
<img width="587" height="240" alt="{FC57B790-AE6C-419E-93FD-FF908BCBAC55}" src="https://github.com/user-attachments/assets/d530a143-2f2b-4f75-8411-4b9079c3ce39" />
<br>
in which, at every time step, the values of Vlon, θ and Ki(cornering stiffness) are updated.


