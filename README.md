This repository is a simulation of the control of a non-linear model of a quadrotor.
Files should be run in the following order:

MathematicalModel: Contains the non linear equations of the quadrator.

LinearizedModel: Linearizes the system by calculating matrices A, B and C, the jacobians of the non linear equations.

GetTrajectory: Get the desired trajectory by user input of waypoints.

FixedGainController: Contains two simulations - a first one where the yaw angle was set to zero, and a second one where the tangential yaw angle was user.

GainSchedulerController: Controls the quadrotor to follow the trajectory with tangential yaw using a gain scheduler controller that recalculates the gain matrices every time that the quadrotor's position moves away from the reference more than a defined tolerance.
