# Introduction:
The modern robotics capstone project deals with robotic manipulation of the KUKA YouBot4. This bot has a holonomic base with 4meccanum wheels attached to the base with a 5DOF arm. To view the deatils of the project, click [here.](http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone)
![Youbot4](https://user-images.githubusercontent.com/45617702/98544270-c38ca280-22b9-11eb-86b6-fb0704d4810a.PNG)

# Index:
- [Milestone1: Odometry](#Milestone1)
- [Milestone2: Trajectory Generation](#Milestone2)
- [Milestone3: PI controller](#Milestone3)
- [Overview](#Overview)

<a name="Milestone1"></a>
# Milestone1:
This part of the project requires to find the next configuration of the robot given the 12vector representation of current orientation.
3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles.

This is acheived by using simple Euler iterations:
![Milestone1eq](https://user-images.githubusercontent.com/45617702/98543538-a73c3600-22b8-11eb-9dd8-7fab669bbb0b.PNG)
