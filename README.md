#Introduction:
The modern robotics capstone project deals with robotic manipulation of the KUKA YouBot4. This bot has a holonomic base with 4meccanum wheels attached to the base with a 5DOF arm. To view the deatils of the project, click [here.](http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone)
![](.data/Youbot4.png)

#Index:
- [Milestone1: Odometry](#Milestone1)
- [Milestone2: Trajectory Generation](#Milestone2)
- [Milestone3: PI controller](#Milestone3)
- [Overview](#Overview)

<a name="Milestone1"></a>
#Milestone1:
This part of the project requires to find the next configuration of the robot given the 12vector representation of current orientation.
_3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles._

This is acheived by using simple Euler iterations:
$ next $ $cconfig$ = $ cconfig+\delta$$Qs*\delta$$t$
$ next$ $joint$ $ angle$ = $current $ $joint$ $ angle$$+joint$ $speed$$*\delta$$t$
$next$ $wheel$ $angle$ = $wheel$ $angle$ $+wheel$ $angle*$$\delta$$t$

Here, $cconfig$- chassis configration
$\delta$$Qs$- Chassis configuartion expressed in space frame as [x, y, $\phi$]
$\delta$$t$- time period of operation. (0.01 seconds)
However the main catch of this exerciseis to evaluate the value of $\delta$$Qs$.
Using wheel velocities we can calculate the base twist $Vb$.
$Vb$ = $\tilde{H}$$(0)*u$
$\tilde{H}$$(0)$- Pseudo inverse of $H(0)$ which is the matrix relating base twist to joint velocities. $H(0)$ matrix:
$$
\frac{1}{r}
\begin{pmatrix}
-l-w & 1 & -1\\
l+w & 1 & 1 \\
l+w & 1 &-1\\
-l-w &1 &1
\end{pmatrix}
\begin{pmatrix}
wbx\\ 
vbx\\
vby
\end{pmatrix}
$$

Now we have the value of $Vb$ in terms of [$wb,vx,vy$] we need to convert it to [$x, y, \phi$].
To convert $Vb$ to $\delta Qb$:
If $wz=0, \delta Qb = vb$
Otherwise,
If $wbz \ne 0$, $\delta Qb =$$
\begin{pmatrix}
wbz\\
\frac{vbx*sin(wbz) +vby (cos(wbz)-1)}{wbz} \\
\frac{vby*sin(wbz) +vbx (1-cos(wbz))}{wbz}\\
\end{pmatrix}
$
The desired value should be converted to a fixed frame {s}. This can be done by using the an appropriate rotation.
$\delta Qs =
\begin{pmatrix}
1 & 0 & 0\\
0 & cos\phi & -sin\phi \\
0 & sin\phi & cos\phi\\
\end{pmatrix}
$