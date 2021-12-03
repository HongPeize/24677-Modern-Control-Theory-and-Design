# CMU 24677-Modern-Control-Theory-and-Design
This repository stores my work in 24677 Modern Control Theory and Design course for fall 2021. There are total 5 projects in this courses. Python is used as programming language and Webot simulator is the simulation environment. I controlled a Tesla Model 3 to achieve self-driving feature by applying different control theory (P1-4) and a DJI unmanned aerial vehicle in P5. <br />
P1: PID Control: Designed a PID longitudinal controller and lateral controller for the vehicle. <br />
P2: Pole Placement: Designed a state feedback lateral controller by using the Pole Placement method. <br />
P3: LQR-A* Path Planning Algorithm: Implemented Linear Quadratic Regulator(LQR) to control the Tesla Model, at the same time, designed an A-star path planning algorithm to perform the obstacle overtaking and re-plan the trajectory. <br />
P4: EKF SLAM: Design an Extended Kalman Filter(EKF) to estimate the global state of the vehicle, including coordinate [X, Y] and heading angle [psi] from [x_dot, y_dot], [psi_dot] on the vehicle frame and range and bearing measurements of map features. <br />
P5: Adaptive Control: Implemented adaptive control theory to augment a LQR-based MPC controller and test its effectiveness in the event of a single quadrotor motor of a DJI UAV experience a 50% loss of thrust during hovering. <br />

Note: Some parameters haven't been tuned to perfection, eg, Q & R penalty matrix in LQR; Kp, Ki, Kd in PID controller. Anyone who sees my codes are welcomed to improve them.
