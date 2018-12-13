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

# import modules
import rospy
import tf
from geometry_msgs.msg import Pose
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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
    # using formula from https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/a1abb738-84ee-48b1-82d7-ace881b5aec0
    px, py, pz = gripper_point
    nx, ny, nz = R_E[0, 2], R_E[1, 2], R_E[2, 2]
    wx = px - gripper_distance * nx
    wy = py - gripper_distance * ny
    wz = pz - gripper_distance * nz

    return wx, wy, wz


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
        # i dont really understand why the values in the walkthrough are different...
        DH_table = {alpha0:     0,  a0:      0,  d1:   0.75,   q1:      q1,
                    alpha1: -pi/2,  a1:   0.35,  d2:      0,   q2: q2-pi/2,
                    alpha2:     0,  a2:  0.125,  d3:      0,   q3:      q3,
                    alpha3: -pi/2,  a3: -0.054,  d4:    1.5,   q4:      q4,
                    alpha4:  pi/2,  a4:      0,  d5:      0,   q5:      q5,
                    alpha5: -pi/2,  a5:      0,  d6:      0,   q6:      q6,
                    alpha6:     0,  a6:      0,  d7:  0.303,   q7:       0}

        #
        #
        # Define Modified DH Transformation matrix
        def create_transformation_matrix(q, a, d, alpha):
            TM = Matrix([[cos(q),             -sin(q),            0,                a],
                         [sin(q)*cos(alpha),   cos(q)*cos(alpha),  -sin(alpha),    -sin(alpha)*d],
                         [sin(q)*sin(alpha),   cos(q)*sin(alpha),   cos(alpha),     cos(alpha)*d],
                         [0,                   0,            0,                1]])
            return TM
        #
        #
        # Create individual transformation matrices
        #
        T0_1 = create_transformation_matrix(q1, a0, d1, alpha0).subs(DH_table)
        T1_2 = create_transformation_matrix(q2, a1, d2, alpha1).subs(DH_table)
        T2_3 = create_transformation_matrix(q3, a2, d3, alpha2).subs(DH_table)
        T3_4 = create_transformation_matrix(q4, a3, d4, alpha3).subs(DH_table)
        T4_5 = create_transformation_matrix(q5, a4, d5, alpha4).subs(DH_table)
        T5_6 = create_transformation_matrix(q6, a5, d6, alpha5).subs(DH_table)
        T6_E = create_transformation_matrix(q7, a6, d7, alpha6).subs(DH_table)
        T0_E = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E)

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
        R_corr = Rot_z(pi)*Rot_y(-pi/2)

        R_E = simplify(R_z * R_y * R_x * R_corr)

        TR0_E = simplify(T0_E * R_E)

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

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                                    [req.poses[x].orientation.x,
                                    req.poses[x].orientation.y,
                                    req.poses[x].orientation.z,
                                    req.poses[x].orientation.w])

            # Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #
            #
            EE = Matrix([[px], [py], [pz]])
            R_E = R_E.subs({'roll': roll, 'pitch': pitch, 'yaw': yaw})
            WC = get_wrist_centre(EE, R_E, 0.303)

            # Calculate joint angles using Geometric IK method
            #
            #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
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
