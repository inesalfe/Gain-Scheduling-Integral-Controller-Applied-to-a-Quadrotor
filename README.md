# Gain Scheduling Intergal Controller Applied to a Quadrotor

This is the final project for the "Robotics" course. This course is a masters course in the degree of Electrical and Compuuter Engeneering in Instituto Superior Técnico - Universidade de Lisboa.

The goal of this project was to develop a Matlab simulation of the control of a non-linear model of a quadrotor. This repository contains the code and the report paper.

## Abstract

The control of non-linear systems presents major challenges when compared to the control of linear systems. To downsize the complexity of non-linear systems, a common practice is to linearize the system around an equilibrium point and, sub- sequently, use known linear control techniques. Problems arise if the state of the system deviates greatly from the equilibrium point around which the system is linearized. To overcome this issue, gain- scheduling control uses different gains, calculated for different equilibrium points. This approach allows for an adaptation of the control parameters depending on the state of the system. Here, we compare a fixed-gain and a gain-scheduling controller by applying them to the problem of tracking a time-varying reference where the non-linear system in consideration is a quadrotor. We experimentally show that a fixed gain controller does not allow, in general, for the quadrotor to follow a time-varying reference. However, this problem is circumvented by applying a gain-scheduling controller.

## Files and usage

Files should be run in the following order:

MathematicalModel: Contains the non linear equations of the quadrator.

LinearizedModel: Linearizes the system by calculating matrices A, B and C, the jacobians of the non linear equations.

GetTrajectory: Get the desired trajectory by user input of waypoints.

FixedGainController: Contains two simulations - a first one where the yaw angle was set to zero, and a second one where the tangential yaw angle was user.

GainSchedulerController: Controls the quadrotor to follow the trajectory with tangential yaw using a gain scheduler controller that recalculates the gain matrices every time that the quadrotor's position moves away from the reference more than a defined tolerance.
