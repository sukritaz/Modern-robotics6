# Introduction:
The modern robotics capstone project deals with robotic manipulation of the KUKA YouBot4. This bot has a holonomic base with 4meccanum wheels attached to the base with a 5DOF arm. To view the deatils of the project, click [here.](http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone)
![Youbot4](https://user-images.githubusercontent.com/45617702/98544270-c38ca280-22b9-11eb-86b6-fb0704d4810a.PNG)

# Index:
- [Milestone1: Odometry](#Milestone1)
- [Milestone2: Trajectory Generation](#Milestone2)
- [Milestone3: PI controller](#Milestone3)
- [Overview](#Overview)
- [Results](#results)

<a name="Milestone1"></a>
# Milestone1:
This part of the project requires to find the next configuration of the robot given the 12vector representation of current orientation.
3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles.

This is acheived by using simple Euler iterations:
![Milestone1eq](https://user-images.githubusercontent.com/45617702/98543538-a73c3600-22b8-11eb-9dd8-7fab669bbb0b.PNG)
Here is an image for all reference frames
![reference](http://hades.mech.northwestern.edu/images/3/33/Yb-book.png)
To understand the formation of matrix q:
![matrixq](https://user-images.githubusercontent.com/45617702/128624267-ad9824ad-3afb-481f-b554-5016750d3450.jpeg)

<a name="Milestone2"></a>
# Milestone2:
A trajectory is a path that is appropriately time scaled. Polynomial time scaling
is a common approach that is adopted. The coefficients of these polynomials are
calculated based on the initial and terminal conditions we subject the motion to.
For the classic case of a pick and place application we have divided the motion of
the robot into 8segments.
1. A trajectory to move the gripper from its initial configuration to a "standoff"
configuration a few cm above the block.
2. A trajectory to move the gripper down to the grasp position.
3. Closing of the gripper.
4. A trajectory to move the gripper back up to the "standoff" configuration.
5. A trajectory to move the gripper to a "standoff" congfiuration above the final
conguration.
6. A trajectory to move the gripper to the final configuration of the object.
7. Opening of the gripper.
8. A trajectory to move the gripper back to the "standoff" configuration.
![traj1](https://user-images.githubusercontent.com/45617702/128625061-40ce377b-b956-44d0-ae1a-f691c875a1a1.png)
![traj2](https://user-images.githubusercontent.com/45617702/128624574-0fe7789e-6993-4613-806a-e3ffa7dc96d9.png)
![traj3](https://user-images.githubusercontent.com/45617702/128624596-4262b65c-7e1f-49af-b1e3-3746d07a8251.png)
![traj4](https://user-images.githubusercontent.com/45617702/128624620-b32657ce-d22e-4834-ab4e-aae73c6a941e.png)

<a name="Milestone3"></a>
# Milestone3:
With the help of our
predefined trajectory, we can calculate the desired twist required to move the
bot from current desired configuration to next desired configuration in unit time
expressed in {s}.
![ff1](https://user-images.githubusercontent.com/45617702/128624689-bded01a8-9f0e-4929-9608-69d4b2515ce4.png)

Where J is the jacobian that relates the end effector twist to joint velocities. This
jacobian is calculated using screw axis theory in robotics.

<a name="Overview"></a>
# Overview: Bringing it together
The structure of the project as per my understanding is as follows:
1. Produce reference trajectory
2. Constantly loop through reference trajectory to get required end-effector twist
3. While looping, run a feedforward PI controller to achieve the required end effector twist
4. Pass this end effector twist to calculate joint velocities
5. With these joint velocities calculate the nextstate of the joints
6. Convert the joint state to Tse, Tse = Tsb * Tbo * Toe
  Where Toe = Fkbody(m,thetalist,blist)
  thetalist is the arm config.
- Remember that the values of o/p from psuedo Jacobian and end effector twist are in the order of Uarm and Ubase, not what we typically considered in Milestone1.

<a name="results"></a>
# Results
![image align="center"](https://user-images.githubusercontent.com/45617702/128625175-24910e13-6bfd-4f69-93ae-c57d8e8badc3.png)
