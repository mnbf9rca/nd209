## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Here's a picture of my picker in action:

[grabber about to pick up a blue tube](./photos/Screenshot 2018-12-18 at 20.27.02.png)

[grabber about to drop the blue tube](./photos/Screenshot 2018-12-18 at 20.27.45.png)
[grabber lining up for another grab](./photos/Screenshot 2018-12-18 at 20.29.19.png)

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I examined the URDF file and created a Denavit-Hartenberg diagram of the robot, which contains 6 revolute joints:

from this diagram, and examination of the URDF file, i derived the folowing DH parameters:
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Transformations are created by the following code method:
```
def create_transformation_matrix(q, a, d, alpha):
    TM = Matrix([[cos(q),           -sin(q),           0,             a],
                    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [0,                 0,           0,             1]])
    return TM
```
these are listed out as follows:
```
T0_1 = create_transformation_matrix(q1, a0, d1, alpha0).subs(DH_table)
T1_2 = create_transformation_matrix(q2, a1, d2, alpha1).subs(DH_table)
T2_3 = create_transformation_matrix(q3, a2, d3, alpha2).subs(DH_table)
T3_4 = create_transformation_matrix(q4, a3, d4, alpha3).subs(DH_table)
T4_5 = create_transformation_matrix(q5, a4, d5, alpha4).subs(DH_table)
T5_6 = create_transformation_matrix(q6, a5, d6, alpha5).subs(DH_table)
T6_E = create_transformation_matrix(q7, a6, d7, alpha6).subs(DH_table)
T0_E = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E
```
the generalised transform from base_link to gripper_link is given by
```
[[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
[ ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
[                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                                 -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
[                                                                                                                                                            0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                                                              1]]
```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

When working out the rotation, there are a few steps. I assumed that WC=J5.
#### theta1: work out the rotation around z of J5
this one's relatively trivial - use ```atan2``` with the provided x,y coordinates given for J5
#### work out the angles theta2, theta3
I found the simplest way to be consider J2/J3/J5 to be a traingle fixed in the plane x,y. Solving this becomes a series of trivial trigonometry steps.
1. Work out the lengths of the sides of the triangle J2/J3/J5

   1. First, adjust the position of J5/WC with respect to J2, where the motion of the arms is along the plane x,y

        1. the x position is given by the combination of the translation along x,y WRT the original reference frame of J1
        2. The y position is simply the z displacement WRT the original reference frame of J1
   
   2. Calculate the length of the line from J3 to J5/WC

        1. Calculate the position of J5 WRT J3. As J4 rotates on the plane x,y, this is the sum of the x and y displacement of J3 to J4 and J4 to J5
        2. The length line_3_5 is the square root of the sum of the x and y displacement of J5 WRT J3
    
    3. Work out the length of the side from J2 to J5/WC as suare root of the sum of the squares of J5/WCx and J5y
    
    4. work out the internal angles of the triangle at J2 and J3, and the angle of the WC/J5 WRT J2

2. theta2 is pi/2 - the rise of J5/WC WRT J2 - the internal angle of J2 in the triangle J5,J2,J3
3. theta3 is pi/2 - the sag of J4 WRT J3 - the internal angle of J3 in the triangle J2,J3,J5
4. Following the instructions in the course, and at https://www.uio.no/studier/emner/matnat/ifi/INF3480/v14/undervisningsmatriale/lec05-Inverse-VelocityKinematicsI.pdf, the inverse kinematic orientation is calculated using a rotation matrix for joints 3, 4, 5 and 6, adjusting for the rotation of joints 1, 2, and 3, and including both the rotational correction for the gripper's z and y axis, and the current roll, pitch and yaw, deriving theta 4, 5 and 6 from teh relative position of the end effector WRT the WC.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


