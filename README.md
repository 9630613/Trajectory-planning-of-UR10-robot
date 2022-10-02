# Trajectory-planning-of-UR10-robot
<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/UR10.png" width= "500"> 
This project is about a six DOF robot called Universal Robot (UR10).This robot contains 6 revolute joints. The robot is going to follow a straight line with constant rotation matrix (the rotation matrix of 6th joint with respect to the base in initial angular configuration ) from the first angular position (qi) to the last one (qf).

$$q_i=\begin{pmatrix} \frac{Pi}{2}\\
0\\
\frac{Pi}{2}\\
0\\
\frac{Pi}{2}\\
\frac{Pi}{6}
\end{pmatrix}              q_f=\begin{pmatrix} \frac{Pi}{4}\\
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
- [Trajectory planning](#Trajectory-planning)                                                                                                                         
- [Invers kinematics](#Invers-kinematics)                                                                                                                            
- [Controlling robot (IK Algorithm)](#Controlling-robot-(IK-Algorithm))                                                                                                
- [VREP](#VREP)                                                                                                                                            
- [Conclusion](#Conclusion)
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
In order to calculate the jacobian matrix, fist of all, the transformation matrix should be made. The following matrix called transformation matrix includes the rotation matrix(   $A_i^{i-1}(q_i) [1:3,1:3]$   ) and the position matrix (   $A_i^{i-1}(q_i) [1:3,4]$   ) that are with respect to their previous joints.


 $$A_i^{i-1}(q_i) =A_i^{i-1}\begin{pmatrix}\alpha_i\\
\theta_i\\
d_i\\
a_i
\end{pmatrix}= A_{i'}^{i-1} A_i^{i'}= \begin{bmatrix}c_{\theta_i} & -s_{\theta_i}c_{\alpha_i} & s_{\theta_i}s_{\alpha_i} & a_ic_{\theta_i}\\
s_{\theta_i} & c_{\theta_i}c_{\alpha_i} & -c_{\theta_i}s_{\alpha_i} & a_is_{\theta_i}\\
0 & s_{\alpha_i} & c_{\alpha_i} & d_i\\
0 & 0 & 0 & 1
\end{bmatrix}$$



According to   $A_i^{i-1}(q_i)$   , the transformation matrix could be calculated. For example, for the first joint is :


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


Now, we should find the position matrix of each joint with respect to the base, that easily could be calculate:


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
In order to calculate the inverse kinematic, it is necessary to use the analytic jacobian which needs the euler angles. The xyz coordinate is used for euler angles because of the VREP coordinate. 
In fact, the only difference between geometric jacobian and analytic jacobian is their angular velocities, whereas they have the same linier velocities, so to calculate the analytic jacobian, the geometric angular velocities should be changed. 
The angular velocities in the geometric jacobian are the derivative of euler angles at every moment.Then, for the (xyz)coordinate :

$$ T_{xyz}\begin{pmatrix} \alpha\\
\theta\\
 \gamma
\end{pmatrix} = \begin{pmatrix} 1& 0 & \sin\theta\\
0& \cos\alpha & -\cos\theta \sin\alpha\\
0& \sin\alpha & \cos\theta \cos\alpha
\end{pmatrix} $$



$$	w= T(\Phi) \Phi'$$

The α, $\theta$, and γ are the euler angles calculated by solving the inverse problem of the rotation matrix of 6th joint with respect to the base at every moment.
In order to caculate the rotation matrix, the elementary rotation matrixes should be multiplied in sequence, $R_x(α), R_y(\theta), R_z(γ)$, and the inverse problem should be solved at the moment. To Solve the problem, the next function is used.

````c++
rotm2eul(R60,’xyz’)
````
As the result, the euler angles are replaced to make T.
Finally, by multiplying T inverse in the angular velocities of geometric jacobian, the analytic one is produced.
Duo to the fact that T should be a $6\times 6$ matrix to be able to be multiplied by U, the extra matrixes $0_{3 \times 3}$ and $I_{3 \times 3}$ (indentity matrix) are added in the nutral positions .

$$ U=\begin{bmatrix} I_{3 \times 3} & 0_{3 \times 3}\\
0_{3 \times 3} & T_{3 \times 3}
\end{bmatrix}$$


$$ J_G=U J_A$$


$$ J_A=\begin{bmatrix} I&0\\
0&T
\end{bmatrix}^{-1}*J_G$$

# Trajectory planning 
To solve the trajectory planning, a linear cartesian path is planned in the 6 dimension task space including position and orientation. Also, a trapezoidal velocity profile is used in timing law to have a constant velocity along the path . 
Now, the initial and final position and orientation should be found.  
The position is calculated by replacing the qi and qf in the end-effector position with respect to the base. The end-effector position is the 4th column in 6th joint of the transformation matrix with respect to the base. thus 
````c++
xi= subs(p60,{q1,q2,q3,q4,q5,q6},{pi/2,0,pi/2,0,pi/2,pi/6}) 
xf= subs(p60,{q1,q2,q3,q4,q5,q6},{pi/4,-pi/4,pi/1.5,0,pi/2,0})
````


$$x_i=\begin{pmatrix} −0.1639\\
 −0.4963\\
0.7918
\end{pmatrix}              q_i=\begin{pmatrix} −0.4645\\
−0.2327 \\
0.3664
\end{pmatrix}$$


The needed orientation is calculated by replacing qi in the rotation matrix of the 6th joint with respect to the base, and finding the euler angles. It is noticable that the final orientation is the same, according to the given data of the issue (constant orientation along the path).
````c++
Rotation matrix=subs(A60(1:3,1:3),{q1,q2,q3,q4,q5,q6},{pi/2,0,pi/2,0,pi/2,pi/6})
Euler angles= rotm2eul (Rotation matrix,’xyz’)
````
Finally, the initial and final position:


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
	
Now, making the trajectory:


$L= ||x_f-x_i||= 0.58m$

$T=7 sec (given)$

$V_{max} =0.1 m/s(consider)$

$T_s= \frac{(T- L)}{V_{max}}$

$a_{max}=\frac{V_{max}}{T_s}$


$$Ϭ=\begin{array}{ccc} 
\frac{a_{𝑚𝑎𝑥}𝑡^2}{2}                          & 	  t \to [0,T_s]\\
v_{𝑚𝑎𝑥}𝑡 − \frac{{v_{𝑚𝑎𝑥}}^2}{2a_{𝑚𝑎𝑥}}                            &    t \to  [T_s,T-T_s]\\
\frac{-𝑎_{𝑚𝑎𝑥}( 𝑡 − 𝑇)^2}{2} + v_{𝑚𝑎𝑥}T+\frac{{v_{𝑚𝑎𝑥}}^2}{a_{𝑚𝑎𝑥}}&        t \to [T-T_s,T]
\end{array}$$
 
 
This timing low will be constant for every desired path with trapezoidal velocity profile, and the difference is in making the path parameterization. The path is linear,so:

$S=\frac{Ϭ}{L}$

$X_{des} = x_i +S(x_f - x_i)$

$d_{X_des}=S(x_f - x_i)$

<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/the%20desired%20velocities.jpg" width= "500">

# Invers kinematics 
Controling the robot is with joint configuration, so q matrix That is the joint configuration at every moment is given.  
$$𝑞′ = 𝐽_{𝐴−1}(𝑞) $$
With integrating from drivative of q, the joint configuration (q) is produced, but there is a problem when a joint is in singularity. As the result, the jacobian loses its rank and we can’t control the robot. That will stops the robot from working or moves it undesirably. thus, the Damped last squares method is used.
$$𝐽^* = 𝐽^T(𝐽 𝐽^T + {𝛾^2}I)^{-1}$$
Now with using a suitable γ , we can control robot in the singularities.
	            	
# Controlling robot (IK Algorithm)
In order to control the robot on the desired path, an error is calculated between desired task and the actual end_effector position and orientation. 
The error is :                 

$$ 𝑒 = 𝑥_{𝑑𝑒𝑠𝑖𝑟𝑒𝑑} − 𝑥_{endeffector} $$

The end-effector position is a 6 by 1 matrix including the position of the 6th joint and euler angles at every moment. Finally:  


$$ 𝑑_𝑞 = 𝐽^T(𝐽 𝐽^T + 𝛾^2{I})^{−1}(𝑑𝑥_{𝑑𝑒𝑠𝑖𝑟𝑒𝑑} − 𝑘 𝐼(𝑥_{𝑑𝑒𝑠𝑖𝑟𝑒𝑑} − 𝑥_{endeffector})) $$


k in this robot 2000.
now the robot is controlled desirability also we can get the error feedback and  decreaes the error very well. 

<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/error%20%20(order10%5E15).jpg" width= "500">

<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/desired%20trajectory.jpg" width= "500">

<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/4the%20joint%20configuration.jpg" width= "500">

by comparing initial and final joint configurations and desired trajectory a very small difference is absorbed because the error is negligibile. 
# VREP 
For connectioning to Vrep we should copy 3 files from VREP file (in installation place) in a new folder and also a VREP and Mtlab window.  
In Matlab we copy some codes and write our codes; the codes are for introduction  the joints to Vrep and also a loop for geting the angular configuration  to the robot in the time ;because we control the robot by configuration of its joints .  
In Vrep window the robot should be chosen and cleared its one code and also inter a code (simRemoteApi.start(19999)) for connectioning to Matlb .  
After that check the joints to be on inverse kinematic mood and turn off the robot motors.    
The important thing in VREP is calibration the joints configuration.  
 the dinavit diagram inVREP is different so we should calibrate the joint to flow the correct path.  
Now we can connect to Vrep and transfer the data from Matlab to Vrep.  

<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/robot%20in%20the%20vrep%20on%20the%20path.jpg" width= "500">

# Conclusion
Controlling robot in this given path needs a strong controller because the singularities and also 
the robot during the path had limit velocity around 0.1 m/sec .
