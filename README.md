# Trajectory-planning-of-UR10-robot
<img src="https://github.com/9630613/Trajectory-planning-of-UR10-robot/blob/main/Images/UR10.png" width= "500"> 
This project is about the six DOF robot which is called Universal Robot (UR10). This robot contains 6 revolute joints. The robot is going to follow a straight line with constant rotation matrix (the rotation matrix of 6th joint with respect to the base in the first angular configuration ) from first angular qi to the last one qf

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


| **DH**       |**ϑ**    |**α**   | **d**      | **a**    |
| -------- |:------:|:-----:|:------:| :-----:|
| A01      |-Pi/2   | q1    |0.1273  |0      |
| A21      | 0      | q2    |0       |-0.612 |
| A32      | 0      | q3    |0       |-0.5723|
| A43      | Pi/2   | q4    |0.1639  |0      |
| A54      | -Pi/2  | q5    |0.1157  |0      |
| A65      | 0      | q6    |0.0922  |0      |

# Geometric jacobian

 $$A_i^{i-1}(q_i) = A_{i'}^{i-1} A_i^{i'} = \begin{bmatrix}c_{\theta_i} & -s_{\theta_i}c_{\alpha_i} & s_{\theta_i}s_{\alpha_i} & a_ic_{\theta_i}\\
s_{\theta_i} & c_{\theta_i}c_{\alpha_i} & -c_{\theta_i}s_{\alpha_i} & a_is_{\theta_i}\\
0 & s_{\alpha_i} & c_{\alpha_i} & d_i\\
0 & 0 & 0 & 1
\end{bmatrix}$$

