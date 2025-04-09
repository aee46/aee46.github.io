+++
title = "Lab 7"
description = "Kalman Filter"
weight = 7

[extra]
remote_image = "/projects/KALMANHEADER.jpg"
+++

Overview
======
This lab focuses on implementing a Kalman Filter to predict distance and speed measurements on the robot. This will be particularly useful in speeding up the the control loop from Lab 5. Instead of using linear extrapolation to predict distance measurements when the TOF sensor isn't ready, we can use a Kalman Filter to do this task more accurately.

Estimating Drag and Momentum
======
The first step in designing and implementing a Kalman Filter involves building the state space model for the robot. This requires estimating drag and the momentum of the robot as it moves towards a wall. This can be done by driving the motors forward with a step response, as seen in the graphs below. In Lab 5, I had limited my control loop to a maximum speed of 50%, corresponding to a PWM value of 127.5. 

NOTE: I did not complete this lab. I ran into issues with my TOF sensors breaking and I also was sick with the flu and did not have the energy to complete this lab (since I had also already used my two slip weeks). 


