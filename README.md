# Trajectory-planning-of-UR10-robot
<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/UR10.png" width= "500"> 
This project is about the six DOF robot called Universal Robot (UR10). This robot contains 6 revolute joints. The robot is going to follow a straight line with constant rotation matrix (the rotation matrix of 6th joint with respect to the base in the first angular configuration ) from first angular qi to the last one qf

$$q_i=\begin{pmatrix} \frac{Pi}{2}\\
0\\
\frac{Pi}{2}\\
0\\
\frac{Pi}{2}\\
\frac{Pi}{6}
\end{pmatrix}              q_i=\begin{pmatrix} \frac{Pi}{4}\\
\frac{-Pi}{4}\\
\frac{Pi}{1.5}\\
0\\
\frac{Pi}{2}\\
0
\end{pmatrix}$$


# Table of content
- [Dinavit Hartenberg](#Dinavit-Hartenberg)                                                                                                                          
- [Geometric jacobian](#Geometric-jacobian)                                                                                                                         
- [Analytic jacobian](#Analytic-jacobian)                                                                                                                           
- Trajectory planning                                                                                                                         
- Invers kinematic                                                                                                                             
- Controlling robot (IK Algorithm)                                                                                                
- VREP                                                                                                                                            
- Conclusion
# Dinavit Hartenberg
The first step is to find DH diagram

<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/DH.jpg" width= "500"> 


|  **DH**       |**$$\theta$$**    |**$$\alpha$$**   | **$d$**      | **$a$**    |
| ------------- |:------:|:-----:|:------:| :-----:|
| $A_{10}$      |$\frac{-Pi}{2}$   | $q_1 $   |0.1273  |0      |
| $A_{21}$      | 0      | $q_2 $     |0       |-0.612 |
| $A_{32}$      | 0      | $q_3 $    |0       |-0.5723|
| $A_{43}$      |$\frac{Pi}{2}$|$q_4 $ |0.1639  |0      |
| $A_{54}$      |$\frac{-Pi}{2}$ |$q_5 $    |0.1157  |0      |
| $A_{65}$      | 0      | $q_6 $     |0.0922  |0      |


# Geometric jacobian
In order to calculate the jacobian matrix, fist of all, the transformation matrix should be made. The following matrix called transformation matrix, included the rotation matrix(   $A_i^{i-1}(q_i) [1:3,1:3]$   ) and the position matrix (   $A_i^{i-1}(q_i) [1:3,4]$   ) that are with respect to their previous joints.


 $$A_i^{i-1}(q_i) =A_i^{i-1}\begin{pmatrix}\alpha_i\\
\theta_i\\
d_i\\
a_i
\end{pmatrix}= A_{i'}^{i-1} A_i^{i'}= \begin{bmatrix}c_{\theta_i} & -s_{\theta_i}c_{\alpha_i} & s_{\theta_i}s_{\alpha_i} & a_ic_{\theta_i}\\
s_{\theta_i} & c_{\theta_i}c_{\alpha_i} & -c_{\theta_i}s_{\alpha_i} & a_is_{\theta_i}\\
0 & s_{\alpha_i} & c_{\alpha_i} & d_i\\
0 & 0 & 0 & 1
\end{bmatrix}$$



According to the   $A_i^{i-1}(q_i)$   , the transformation matrix could be calculated. For example, for first joint is :


$$A_{10}=A_1^{0}\begin{pmatrix}\alpha_1\\
\theta_1\\
d_1\\
a_1
\end{pmatrix} =A_1^{0}\begin{pmatrix} q_1\\
\frac{-Pi}{2}\\
0.1273\\
0
\end{pmatrix}= \begin{bmatrix}c(\frac{-Pi}{2}) & -s(\frac{-Pi}{2})c(q_1) & s(\frac{-Pi}{2})s(q_1) & 0 c(\frac{-Pi}{2})\\
s(\frac{-Pi}{2}) & c(\frac{-Pi}{2})c(q_1) & -c(\frac{-Pi}{2})s(q_1) & 0 s(\frac{-Pi}{2})\\
0 & s(q_1) & c(q_1) & 0.1273\\
0 & 0 & 0 & 1
\end{bmatrix}$$


Other joints are calculated as the same.



$$ A_{21}=A_2^{1}\begin{pmatrix} q_2\\
0\\
0\\
-0.612
\end{pmatrix}$$


$$A_{32}=A_3^{2}\begin{pmatrix} q_3\\
0\\
0\\
-0.5723
\end{pmatrix}$$

$$A_{43}=A_4^{3}\begin{pmatrix} q_4\\
\frac{Pi}{2}\\
0.163941\\
0
\end{pmatrix}$$

$$A_{54}=A_5^{4}\begin{pmatrix} q_5\\
\frac{-Pi}{2}\\
0.1157\\
0
\end{pmatrix}$$

$$A_{65}=A_6^{5}\begin{pmatrix} q_6\\
0\\
0.0922\\
0
\end{pmatrix}$$




Now, by multiplying each matrix by its all privious ones, the trnsformation matrix of each joint with respect to the base could be produced.



$A_{20} = A_{10} A_{21}$



$A_{30} = A_{10} A_{21} A_{32}$



$A_{40} = A_{10} A_{21} A_{32} A_{43}$



$A_{50} = A_{10} A_{21} A_{32} A_{43}*A_{54}$



$A_{60} = A_{10} A_{21} A_{32} A_{43} A_{54} A_{65}$



Afterwards, the position matrix of the last joint(6th) with respect to other joints is produced like:

$P_{60} = A_{60}[1:3,4]$


$P_{61} = A_{60}[1:3,4]-A_{10}[1:3,4]$


$P_{62} = A_{60}[1:3,4]-A_{20}[1:3,4]$


$P_{63} = A_{60}[1:3,4]-A_{30}[1:3,4]$


$P_{64} = A_{60}[1:3,4]-A_{40}[1:3,4]$


$P_{65} = A_{60}[1:3,4]-A_{50}[1:3,4]$


One last part is to find the position matrix of each joint with respect to the base, that easily could be calculate:


$$Z_0=\begin{bmatrix} 0\\
0\\
1
\end{bmatrix}$$


$Z_1=A_{10}[1:3,4]$

$Z_2=A_{20}[1:3,4]$

$Z_3=A_{30}[1:3,4]$

$Z_4=A_{40}[1:3,4]$

$Z_5=A_{50}[1:3,4]$

Finally, this is ready to replace the elements in jacobian equasion.


$$J=\begin{bmatrix} Z_0 \times P_{60} & Z_1 \times P_{61} & Z_2 \times P_{62} & Z_3 \times P_{63} & Z_4 \times P_{64} & Z_5 \times P_{65} \\
 Z_0 &  Z_1 &  Z_2 & Z_3 &  Z_4 & Z_5
\end{bmatrix}$$


# Analytic jacobian
For invers kinematic is needed to use the analytic jacobian ,which  needs the euler angles  We use the xyz coordinate because of the VREP coordinate. 
We know that the difference between geometric and analytic jacobian is in the angular velocities and they have the same linier velocities so for calculating analytic jacobian the geometric angular velocities  should be changed. 
The angular velocities in geometric jacobian are the derivative of euler angles in every moment Then  for the (xyz)coordinate have 

$$ T_{xyz}\begin{pmatrix} \alpha\\
\theta\\
 \gamma
\end{pmatrix} = \begin{pmatrix} 1& 0 & \sin\theta\\
0& \cos\alpha & -\cos\theta \sin\alpha\\
0& \sin\alpha & \cos\theta \cos\alpha
\end{pmatrix} $$



$$	w= T(\Phi) \Phi'$$

The α, \theta, γ are the euler angles wich are calculated them by solving the invers problem rotation 
matrix 6
th joint with respect to the base in every moment.
so we should multiply elementary rotation matrix sequence $R_x(α), R_y(\theta), R_z(γ)$ and solve the 
invers problem in each moment for this the function (rotm2eul(R60,’xyz’) )is used in Matlab
that gives euler angles in the moment and we can calculate T matrix.
Finally by multiplying T inverse in the angular velocities of geometric jacobian , get the analytic 
one.
because we want to have a product of matrix we use the 6 by 6 matrix U.

$$ U=\begin{bmatrix} I&0\\
0&T
\end{bmatrix}$$


$$ J_G=U J_A$$


As you see the I matrix is a 3by3 indentity matrix and 0 matrix is a 3by 3 zero  matrix. thus  


$$ J_A=\begin{bmatrix} I&0\\
0&T
\end{bmatrix}^{-1}*J_G$$

# Trajectory planning 
For trajectory planning we are planning a linier cartesian path in the 6 dimension task space wich is contains position and orientation ; also we use a trapezoidal velocity profile in timing law to have a constant velocity along the path . 
We find the initial and final position and orientation.  
The position is calculated by replacing the qi and qf in the end-effector position with respect to the base that is the 4th column in 6th joint transformation matrix with respect to the base thus 
````c++
xi= subs(p60,{q1,q2,q3,q4,q5,q6},{pi/2,0,pi/2,0,pi/2,pi/6}) 
xf= subs(p60,{q1,q2,q3,q4,q5,q6},{pi/4,-pi/4,pi/1.5,0,pi/2,0})
````
thus 

$$x_i=\begin{pmatrix} −0.1639\\
 −0.4963\\
0.7918
\end{pmatrix}              q_i=\begin{pmatrix} −0.4645\\
−0.2327 \\
0.3664
\end{pmatrix}$$


orientation is needed wich is calculated by replacing qi in the the 6th joint rotation matrix with 
respect to the base and found the euler angles, it is notice that the final orientation is the same 
because the constant orientation along the path is desired.
````c++
Rotation matrix=subs(A60(1:3,1:3),{q1,q2,q3,q4,q5,q6},{pi/2,0,pi/2,0,pi/2,pi/6})
Euler angles= rotm2eul (Rotation matrix,’xyz’)
````
Finally we have the initial and final pose:


$$x_i=\begin{pmatrix} −0.1639\\
 −0.4963\\
0.7918\\
0\\
0\\
−2.6118
\end{pmatrix}              q_i=\begin{pmatrix} −0.4645\\
−0.2327 \\
0.3664\\
0\\
0\\
−2.6118
\end{pmatrix}$$
	
Now, we can make our trajectory:

$ L= ||x_f-x_i||= 0.58m $

$T=7 sec (given)$

$V_{max} =0.1 m/s(consider)$

$T_s= \frac{(T- L)}{V_{max}}$

$a_{max}=\frac{V_{max}}{T_s}$




	            	

