# Advanced Control System

Exploration of robot dynamics and control architectures. The project contains a full analysis of an assigned 3dof robot (RPP):

- Kinematics (Direct, Inverse, Differential)
- Dynamics (Lagrangian, RNE formulations)
- Model Linearization

The following control architectures are implemented in Matlab Simulink:

- Joint Space PD with gravity compensation
- Joint Space Inverse Dynamics
- Operational Space PD with gravity compensation
- Operational Space Inverse Dynamics
- Compliance Control
- Impendance Control
- Admittance Control
- Force with inner position loop
- Parallel Force control
- Adaptive Control (for a 1 dof link for simplicity)

TODO:

- [ ] Understand the problem with Compliance control
- [ ] Investigate in a better way the force architectures joints/operational meaning (where is the env/ the force applied)
- [ ] Check again Admittance and Impendance to figure out improvements
- [ ] Impedance control is slow due to lots of pinv ed numerical problems with orientations
- [ ] Understand a bit better force control architectures
- [ ] Complete the report with all the control architectures
- [ ] Implement the model linearization and report it
- [ ] Check again all the assignment before the exam
- [ ] Complete reports with all images/scenarios and check kinematics
- [ ] Adaptive check why it doesn't work with sin wave function and it doesn't keep the estimation