# Pick and Place youbot simulation

## Description
DEveloped a software that plans a trajectory for the end-effector of the youBot mobile manipulator (a mobile base with four mecanum wheels and a 5R robot arm), performs odometry as the chassis moves, and performs feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down.

## Note: All the code should be run from the Ghosh_Ritika_capstone directory, the instructions
are provided in the codes and also in their respective README.pdf. Each of the milestones
with the TrajectoryGenerator(), NextState() and FeedbackControl() function are included in
the code directory in milestone1.py, milestone2.py and milestone3.py respectively. They are
imported into the best.py, overshoot.py and new_task.py.

## Overview:
This software plans a trajectory for the end-effector of the youBot mobile manipulator (a
mobile base with four mecanum wheels and a 5R robot arm), performs odometry as the
chassis moves, and performs feedback control to drive the youBot to pick up a block at a
specified location, carry it to a desired location, and then puts it down.
The initial configuration used is (x, y, 𝜃) = (0.2, 0, π/3), i.e the chassis phi, chassis x, chassis
y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state = ( π/3, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
The default initial block configuration is at (x, y, 𝜃) = (1m, 0m, 0 rad) and the final block
configuration is at (x, y, 𝜃) = (0 m,-1m, -π/2 rad). Incase of ‘New Task’ initial block
configuration is at at (x, y, 𝜃) = (1.5m, 1.0m, 0 rad) and the final block configuration is at (x,
y, 𝜃) = (0m, -1.5m, -π/2rad).

The best results were obtained using a PI feedback controller with gains Kp = 12 and Ki =
0.5. The graph converges to a straight line which is as was expected from this PI
controller.The robot can be seen correcting its orientation at the beginning making the initial
error nearly 0.
The overshoot results were obtained using a PI feedback controller with gains Kp = 2.5 and
Ki = 1.5. The result was as expected, where the robot moves a little too far when it reaches
the initial standoff position of the box and then readjust to the correct place, the graph reflects
this behavior.
In New Task, as the position of picking and placing, the robot reaches the correct new
position, however the graph, although it converges, has a small error for a very brief time.