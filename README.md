# Pick and Place youbot simulation

## Description
Developed a software that plans a trajectory for the end-effector of the youBot mobile manipulator (a mobile base with four mecanum wheels and a 5R robot arm), performs odometry as the chassis moves, and performs feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down.

### Note: 
All the code should be run from the Ghosh_Ritika_capstone directory, the instructions
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

https://user-images.githubusercontent.com/60728026/208183298-700ca032-b5fe-4c3c-9e81-28e1b22d2cd8.mp4

The best results were obtained using a PI feedback controller with gains Kp = 12 and Ki =
0.5. The graph converges to a straight line which is as was expected from this PI
controller.The robot can be seen correcting its orientation at the beginning making the initial
error nearly 0.

![X_error BEST Kp= 12 and Ki= 0 5](https://user-images.githubusercontent.com/60728026/208183562-f86df0c7-7e17-463c-bdf7-d1d732298a8c.png)

![X_error BEST display Kp= 12 and Ki= 0 5](https://user-images.githubusercontent.com/60728026/208183475-eec41795-d16e-493e-8711-4a3ebcaa6536.png)

The overshoot results were obtained using a PI feedback controller with gains Kp = 2.5 and
Ki = 1.5. The result was as expected, where the robot moves a little too far when it reaches
the initial standoff position of the box and then readjust to the correct place, the graph reflects
this behavior.

https://user-images.githubusercontent.com/60728026/208184031-0cf2cff8-301a-4619-938b-3a6e792a27ae.mp4

![X_error OVERSHOOT Kp= 2 5 and Ki= 1 5](https://user-images.githubusercontent.com/60728026/208183755-82eacf18-f039-4bf7-aee4-113a0252c729.png)

![X_error OVERSHOOT display Kp= 2 5 and Ki= 1 5](https://user-images.githubusercontent.com/60728026/208183694-fab02e29-3151-43eb-ae7f-b3651cded229.png)

In New Task, as the position of picking and placing, the robot reaches the correct new
position, however the graph, although it converges, has a small error for a very brief time.

https://user-images.githubusercontent.com/60728026/208184105-4d9b1ee7-5ca3-4396-8124-4b3fa08cf757.mp4

![X_error New Task Kp= 12 and Ki= 0 5](https://user-images.githubusercontent.com/60728026/208183871-cd11bcda-5560-4237-8291-4483fb8495f5.png)

![X_error New Task display Kp= 12 and Ki= 0 5](https://user-images.githubusercontent.com/60728026/208183825-7bf96bd6-3f34-411a-bd59-4f6b2bfab83d.png)