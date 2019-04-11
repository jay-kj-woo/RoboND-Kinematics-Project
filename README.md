[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

[//]: # (Image References)

[urdf]: ./misc_images/urdf.png
[DHparam]: ./misc_images/DH_reference_original.jpeg
[KukaArm]: ./misc_images/misc2.png
[Inverse_q23]: ./misc_images/misc3.png
[Inverse_q1]: ./misc_images/inverse_q1.png
[sim init]: ./misc_images/gazebo_init.jpg
[sim pick]: ./misc_images/gazebo_pick.jpg
[sim drop]: ./misc_images/gazebo_drop.jpg


# Robotic arm - Pick & Place project for Kuka KR210

---
This project uses Kuka KR210, 6 DOF serial manipulator, to pick up a cylinder from a shelf and drop it into a bin next to the manipulator. The location of the cylinder is known ahead and the inverse kinematic is performed to calculate the robot joint angles to pick this cylinder up. 

![image of robot intro][KukaArm]



## Kinematic Analysis
### Denavit-Hartenberg Diagram
The DH parameter reference frame is shown in the below image.
![image of DH reference][DHparam]

The reference frame assigned in URDF file is shown in the below image.
![image of DH urdf reference][urdr]

### Denavit-Hartenberg Parameter
After performing the kinematic analys, the following DH parameters are derived. 

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | ---: | ---: | ---: | ---:
0->1 | 0 | 0 | 0.75 | qi
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

### Individual Joint Transformation Matrix
Transformation matrices for individual joints are calculated by substuting the above DH parameters into the modified DH transformation matrix for which a function is defined for convenience. 

```python
def TF_matrix(alpha, a, d, q):
        TF = Matrix([[           cos(q),           -sin(q),           0,             a],
                     [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                     [                0,                 0,           0,             1]])
        return TF
```
Then the individual transformation matrices are found by simple substitution

```python
T0_1 = TF_matrix(alpha0, a0, d1, q1).subs(dh)
T1_2 = TF_matrix(alpha1, a1, d2, q2).subs(dh)
T2_3 = TF_matrix(alpha2, a2, d3, q3).subs(dh)
T3_4 = TF_matrix(alpha3, a3, d4, q4).subs(dh)
T4_5 = TF_matrix(alpha4, a4, d5, q5).subs(dh)
T5_6 = TF_matrix(alpha5, a5, d6, q6).subs(dh)
T6_G = TF_matrix(alpha6, a6, d7, q7).subs(dh)
```
The results are:
```
T0_1 = ([
[cos(q1), -sin(q1), 0,    0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]])


T1_2 = ([
[sin(q2),  cos(q2), 0, 0.35],
[      0,        0, 1,    0],
[cos(q2), -sin(q2), 0,    0],
[      0,        0, 0,    1]])


T2_3 = ([
[cos(q3), -sin(q3), 0, 1.25],
[sin(q3),  cos(q3), 0,    0],
[      0,        0, 1,    0],
[      0,        0, 0,    1]])


T3_4 = ([
[ cos(q4), -sin(q4), 0, -0.054],
[       0,        0, 1,    1.5],
[-sin(q4), -cos(q4), 0,      0],
[       0,        0, 0,      1]])


T4_5 = ([
[cos(q5), -sin(q5),  0, 0],
[      0,        0, -1, 0],
[sin(q5),  cos(q5),  0, 0],
[      0,        0,  0, 1]])


T5_6 = ([
[ cos(q6), -sin(q6), 0, 0],
[       0,        0, 1, 0],
[-sin(q6), -cos(q6), 0, 0],
[       0,        0, 0, 1]])


T6_G = ([
[1, 0, 0,     0],
[0, 1, 0,     0],
[0, 0, 1, 0.303],
[0, 0, 0,     1]])

```
When the robot is in home position (i.e. all the joint angles are zeros), the resultant tranform between the base_link and the gripper_link is as follows
```
T0_G at home = ([
[0,  0,  1,  2.153], 
[0, -1,  0,  	 0], 
[1,  0,  0,  1.946], 
[0,  0,  0,      1]])
```


### Inverse Kinematics
From the known cylinder location, the corresponding wrist center (WC) location and orientation can be obtained.
Then the inverse kineamtics is performed to get the joint angles. 
Joint 1, 2, and 3 angles are calculated using geometric inverse kinematic method.
##### Joint 1
From x-y plane view, joint 1 can be easily calculated.
![image for inverse q1][Inverse_q1]
```python
q1 = atan2(WCy, WCx)
```
#### Joint 2 and 3
For joint 2 and 3, the following diagram is used. 
![image for inverse q23][Inverse_q23]
The calculation of joint angle 2 and 3 are shown in the belwo code
```python
sideA = 1.501
sideB = sqrt((sqrt(WCx**2 + WCy**2) -0.35)**2 + (WCz-0.75)**2)
sideC = 1.25
angleA = acos((sideB**2 + sideC**2 - sideA**2)/(2*sideB*sideC))
angleB = acos((sideA**2 + sideC**2 - sideB**2)/(2*sideA*sideC))
angleWC2base = atan2((WCz-0.75),sqrt(WCx**2 + WCy**2) - 0.35)
angleWC2link3 = atan2(0.054,1.5)   
theta2 = pi/2 - angleA - angleWC2base
theta3 = pi/2 - angleB - angleWC2link3  
```
#### Joint 4, 5, and 6
Since the final orientation of the joints, `R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6`, should be equal to the orientation of the gripper, `R_G`, we can use the following equations to calculate the angles for joint 4, 5, and 6.
```
R3_6 = inverse(R0_3) * R_G
```
With the calculated joint angles for 1, 2, and 3 subsituted, it is possible to get `R0_3`. 
Then the rest is done in the following code
```python
R3_6 = R0_3.transpose() * R_G
theta4 = atan2( R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt((R3_6[0,2])**2 + (R3_6[2,2])**2), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```
## Project Implementation
The above inverse kinematic analysis is implemented to [IKdebug](RoboND-Kinematics-Project/IK_debug.py) to evalulate the accuracy to the known solutions.
The full implementation for ROS environment is made to [IKserver](RoboND-Kinematics-Project/kuka_arm/scripts/IK_server.py).
The result from one of the pick-and-place tasks is shown below
![initial state][sim init]
initial state of the robot and the cylinder

![pick state][sim pick]
robot successfully traveled to the cylinder pick up location

![drop state][sim drop]
robot successfully traveled to the dropping bin location

