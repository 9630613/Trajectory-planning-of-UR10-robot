# Trajectory-planning-of-UR10-robot
<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/UR10.png" width= "500"> 
This project is about the six DOF robot called Universal Robot (UR10). This robot contains 6 revolute joints. The robot is going to follow a straight line with constant rotation matrix (the rotation matrix of 6th joint with respect to the base in the first angular configuration ) from first angular qi to the last one qf

qi =[ pi/2       0          pi/2       0        pi/2   pi/6] 

qf  =[ pi/4   -pi/4      pi/1.5      0       pi/2     0   ] 
# Table of content
- Dinavit Hartenberg                                                                                                                          
- Geometric jacobian                                                                                                                         
- Analytic jacobian                                                                                                                           
- Trajectory planning                                                                                                                         
- Invers kinematic                                                                                                                             
- Controlling robot (IK Algorithm)                                                                                                
- VREP                                                                                                                                            
- Conclusion
# Dinavit Hartenberg
The first step is to find DH diagram

<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/DH.jpg" width= "500"> 


|  **DH**       |**ϑ**    |**α**   | **d**      | **a**    |
| ------------- |:------:|:-----:|:------:| :-----:|
| $A_{10}$      |$\frac{-Pi}{2}$   | $q_1 $   |0.1273  |0      |
| $A_{21}$      | 0      | $q_2 $     |0       |-0.612 |
| $A_{32}$      | 0      | $q_3 $    |0       |-0.5723|
| $A_{43}$      |$\frac{Pi}{2}$|$q_4 $ |0.1639  |0      |
| $A_{54}$      |$\frac{-Pi}{2}$ |$q_5 $    |0.1157  |0      |
| $A_{65}$      | 0      | $q_6 $     |0.0922  |0      |


# Geometric jacobian
Inorder to calculate the jacobian matrix, fist of all, the transformation matrix should be made. The following matrix called transformation matrix, included the rotation matrix(   $A_i^{i-1}(q_i) [1:3,1:3]$   ) and the position matrix (   $A_i^{i-1}(q_i) [1:3,4]$   ) that are taking into account with respect to their previous joint.


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



$A_{20} = A_{10}*A_{21}$



$A_{30} = A_{10}*A_{21}*A_{32}$



$A_{40} = A_{10}*A_{21}*A_{32}*A_{43}$



$A_{50} = A_{10}*A_{21}*A_{32}*A_{43}*A_{54}$



$A_{60} = A_{10}*A_{21}*A_{32}*A_{43}*A_{54}*A_{65}$











A20=A10*A21              	   
A30=A10*A21*A32 
A40=A10*A21*A32*A43 
A50=A10*A21*A32*A43*A54 A60=A10*A21*A32*A43*A54*A65 p60=A60(1:3,4) p61=A60(1:3,4)-A10(1:3,4) p62=A60(1:3,4)-A20(1:3,4) p63=A60(1:3,4)-A30(1:3,4) p64=A60(1:3,4)-A40(1:3,4) p65=A60(1:3,4)-A50(1:3,4) 
Z0=[0;0;1] 
Z1=A10(1:3,3) 
Z2=A20(1:3,3) 
Z3=A30(1:3,3) 
Z4=A40(1:3,3) 
Z5=A50(1:3,3) 




