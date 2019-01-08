#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

from mpmath import *
from sympy import *
#  import numpy as np

# import modules
import rospy
import tf
from geometry_msgs.msg import Pose
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# conversions
rtd = 180 / pi
dtr = pi / 180


def create_R_x(q):
    R_x = Matrix([[1,       0,        0],
                  [0,  cos(q),  -sin(q)],
                  [0,  sin(q),   cos(q)]])
    return R_x


def create_R_y(q):
    R_y = Matrix([[cos(q),       0,   sin(q)],
                  [0,       1,        0],
                  [-sin(q),       0,   cos(q)]])
    return R_y


def create_R_z(q):
    R_z = Matrix([[cos(q), -sin(q),        0],
                  [sin(q),  cos(q),        0],
                  [0,       0,        1]])
    return R_z


def get_wrist_centre(gripper_point, R_E, gripper_distance):
    px, py, pz = gripper_point
    nx, ny, nz = R_E[0, 2], R_E[1, 2], R_E[2, 2]

    wx = px - gripper_distance * nx
    wy = py - gripper_distance * ny
    wz = pz - gripper_distance * nz

    return wx, wy, wz


def calculate_angle(opposite_side, side_b, side_c):
    '''following cosine rule, calculates the angle opposite opposite_side'''
    return acos((side_b * side_b + side_c * side_c - opposite_side * opposite_side) /
                (2 * side_b * side_c))


def calculate_side(opposite_angle, side_b, side_c):
    '''given two adjacent sides and the opposite angle of any triangle,
       calculates the length of the missing side using cosine rule'''
    return sqrt(side_b*side_b + side_c*side_c - 2*side_b*side_c*cos(opposite_angle))


def calculate_hypotenuse(side_a, side_b):
    '''given the length of the opposite and adjacent sides of a right angled triangle,
       calculates the hypotenuse'''
    return sqrt(side_a*side_a + side_b*side_b)


def ensure_rotation_around_zero(angle_to_check):
    '''checks if an angle is > pi/2 and if so, inverts it'''
    if angle_to_check > pi/2:
        return pi/2 - angle_to_check
    return angle_to_check


def clip_value(value, upper, lower):
    if value > upper:
        return upper
    elif value < lower:
        return lower
    return value


def prevent_exceeding_180_degrees(angle):
    '''checks if an angle is > pi, and if so, flips it so the
       arm travels the other way around.
       for exmaple, 185 degrees becomes -175 degrees'''
    if angle > pi:
        print("angle %s is greater than pi", angle)
        return -pi - (angle - pi)
    elif angle < -pi:
        print("angle %s is less than -pi", angle)
        return pi + (angle + pi)
    print("angle %s is within pi > x > -pi", angle)
    return angle


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Your FK code here
        # Create symbols

        # symbols for joint angles
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # twist angles
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # joint angles - theta_i

        #
        #
        # Create Modified DH parameters

        DH_table = {alpha0:     0,  a0:      0,  d1:   0.75,   q1:      q1,
                    alpha1: -pi/2,  a1:   0.35,  d2:      0,   q2: q2-pi/2,
                    alpha2:     0,  a2:   1.25,  d3:      0,   q3:      q3,
                    alpha3: -pi/2,  a3: -0.054,  d4:    1.5,   q4:      q4,
                    alpha4:  pi/2,  a4:      0,  d5:      0,   q5:      q5,
                    alpha5: -pi/2,  a5:      0,  d6:      0,   q6:      q6,
                    alpha6:     0,  a6:      0,  d7:  0.303,   q7:       0}

        #
        #
        # Define Modified DH Transformation matrix
        def create_transformation_matrix(q, a, d, alpha):
            TM = Matrix([[cos(q),           -sin(q),           0,             a],
                         [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                         [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                         [0,                 0,           0,             1]])
            return TM
        #
        #
        # Create individual transformation matrices
        #
        T0_1 = create_transformation_matrix(q1, a0, d1, alpha0).subs(DH_table)
        T1_2 = create_transformation_matrix(q2, a1, d2, alpha1).subs(DH_table)
        T2_3 = create_transformation_matrix(q3, a2, d3, alpha2).subs(DH_table)
        # matrix below only used in ik_debug.py
        # T3_4 = create_transformation_matrix(q4, a3, d4, alpha3).subs(DH_table)
        # T4_5 = create_transformation_matrix(q5, a4, d5, alpha4).subs(DH_table)
        # T5_6 = create_transformation_matrix(q6, a5, d6, alpha5).subs(DH_table)
        # T6_E = create_transformation_matrix(q7, a6, d7, alpha6).subs(DH_table)
        # T0_E = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E

        #
        # Extract rotation matrices from the transformation matrices
        #
        #
        ###
        roll, pitch, yaw = symbols('roll, pitch, yaw')

        R_x = create_R_x(roll)
        R_y = create_R_y(pitch)
        R_z = create_R_z(yaw)
        # translation to URDF reference frame is 180 (pi) deg around Z and -90 (-pi/2) around Y
        R_zyx = R_z * R_y * R_x

        R_corr = create_R_z(pi)*create_R_y(-pi/2)

        R_E = R_zyx * R_corr
        print("R_E", R_E)

        # calculate length of J3-J5 (outside for loop as is static)
        # also calculate the sag of link 4 WRT horizontal from J3

        # start with the position of J4 and J5 from the URDF
        J4 = [0.96, -0.054]
        J5 = [0.96 + 0.54, -0.054]

        # calculate length of the shortest line from J3 to J5
        line_3_5 = calculate_hypotenuse(J5[0], J5[1])

        # work out the angle that link_3 sags - this is the angle which J4 is down WRT J3
        angle_link_4 = atan2(J4[1], J4[0])


        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (r, p, y) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x,
                 req.poses[x].orientation.y,
                 req.poses[x].orientation.z,
                 req.poses[x].orientation.w])

            # Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #
            #
            EE = Matrix([[px], [py], [pz]])
            rpy_table = { roll: r, 
                         pitch: p,
                           yaw: y}
            R_E = R_E.subs(rpy_table)
            wx, wy, wz = get_wrist_centre(EE, R_E, 0.303)  # wx, wy, wz

            # Calculate joint angles using Geometric IK method
            #
            #

            # theta1 = translation around z plane of y and x
            theta1 = atan2(wy, wx)

            # adjust WC to reference frame where J2 is 0,0
            WC_WRT_J2 = (sqrt(wx**2 + wy**2) - 0.35, wz - 0.33 - 0.42)

            # we have the size of the sides of the triangle J2-J3 and J3-J5 from above
            # now, work out the size of the side J2-J5 (J5 = WC)
            length_J2_to_WC = calculate_hypotenuse(WC_WRT_J2[0], WC_WRT_J2[1])
            # work out angle of J3 from J2
            link_2 = 1.25  # link_2 goes from J2 to J3

            # calculate the internal angle at J2 inside the triangle J3-J2-J5
            internal_angle_of_J2 = calculate_angle(line_3_5, link_2, length_J2_to_WC)
            # calculate the internal angle at J3 inside the triangle J2-J3-J5
            internal_angle_of_J3 = calculate_angle(length_J2_to_WC, link_2, line_3_5)
            # work out angle of J5 (WC) from J2
            angle_of_WC_from_J2 = atan2(WC_WRT_J2[1], WC_WRT_J2[0])
            # subtract the internal angles from pi/2 to give the rotation of theta2
            theta2 = (pi/2 - angle_of_WC_from_J2 - internal_angle_of_J2)
            # subtract the internal angle for J3 from pi/2, and subtract the sag of link 4 WRT J3
            theta3 = (pi/2 - internal_angle_of_J3 - angle_link_4)

            # J5,J7,J7 are given by the inverse of the translation of J1,J2,J3 WRT the roll, pitch and yaw of the end effector
            # from https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/a1abb738-84ee-48b1-82d7-ace881b5aec0
            # and https://www.uio.no/studier/emner/matnat/ifi/INF3480/v14/undervisningsmatriale/lec05-Inverse-VelocityKinematicsI.pdf
            R0_3 = (T0_1 * T1_2 * T2_3)[0:3, 0:3]

            R3_6 = (R0_3.T * R_zyx * R_corr).evalf(subs={q1: theta1,
                                                         q2: theta2,
                                                         q3: theta3,
                                                         roll: r,
                                                         pitch: p,
                                                         yaw: y})

            # extract angles from rotated matrix
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1.evalf(), 
                                                theta2.evalf(), 
                                                theta3.evalf(),
                                                theta4.evalf(),
                                                theta5.evalf(),
                                                theta6.evalf()]

            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
