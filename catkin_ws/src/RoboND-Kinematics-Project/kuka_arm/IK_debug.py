from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[-2.432, 0.843, 2.166],
                  [0.366, -0.041, 0.737, 0.567]],
                  [-2.415, 0.688, 2.052],
                  [2.86,0.58,-0.78,-2.17,1.14,-4.20]],
              5:[]}
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
    '''given two adjacent sides and the opposite angle, calculates the length of the missing side'''
    return sqrt(side_b*side_b + side_c*side_c - 2*side_b*side_c*cos(opposite_angle))

def calculate_hypotenuse(side_a, side_b):
    return sqrt(side_a*side_a + side_b*side_b)

def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 
 
    ## Insert IK code here!
    # symbols for joint angles
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link offset along x on xy plane
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link length along z on zy plane
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # twist angles
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # joint angles - theta_i

    #
    #
    # Create Modified DH parameters

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
        TM = Matrix([[            cos(q),           -sin(q),           0,             a ],
                        [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d ],
                        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d ],
                        [                 0,                 0,           0,             1 ]])
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
    T0_E = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E

    ## outside loop
    roll, pitch, yaw = symbols('roll, pitch, yaw')

    R_x = create_R_x(roll)
    R_y = create_R_y(pitch)
    R_z = create_R_z(yaw)
    # translation to URDF reference frame is 180 (pi) deg around Z and -90 (-pi/2) around Y
    R_E = R_z * R_y * R_x
    
    R_corr = create_R_z(pi)*create_R_y(-pi/2)

    R_E = simplify(R_E * R_corr)
    print("R_E", R_E)

    # calculate length of J3-J5 (outside for loop as is static)
    J4 = [0.96, -0.054]
    J5 = [0.96 + 0.54, -0.053]
    link_4 = 0.54
    link_3 = sqrt(J4[0]**2 + J4[1]**2)
    angle_of_line_from_3_to_5 = atan2(J5[1], J5[0])
    angle_link_4 = atan2(J4[1], J4[0])
    angle_between_link_4_and_line_3_5 = angle_of_line_from_3_to_5 - angle_link_4
    line_3_5 = sqrt(link_4**2 + link_3**2 + 2 * link_4 * link_3 * cos(angle_between_link_4_and_line_3_5))
    print("line_3_5", line_3_5, "angle_between_link_4_and_line_3_5", angle_between_link_4_and_line_3_5)

    

    ## per loop

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
    wx, wy, wz = get_wrist_centre(EE, R_E, 0.303) # wx, wy, wz
    print("WC",wx, wy, wz)

    # Calculate joint angles using Geometric IK method
    #
    #
    ###
    theta1 = atan2(wy, wx)

    # first, calculate the length of the 3 sides of the triangle J2,J3,J5(WC)

    # adjust WC to reference frame where J2 is 0,0
    WC_WRT_J2 = (wx - 0.35, wz - 0.33 - 0.42)
    print("WC_WRT_J2", WC_WRT_J2)

    # work out angle of WC from J2
    angle_of_WC_from_J2 = atan2(WC_WRT_J2[1], WC_WRT_J2[0])
    length_J2_to_WC = sqrt(WC_WRT_J2[0]**2 + WC_WRT_J2[1]**2)
    # work out angle of J3 from J2
    link_2 = 1.25 # link_2 goes from J2 to J3
    internal_angle_of_J3_from_J2 = acos((length_J2_to_WC**2 + link_2**2 - line_3_5**2) / (2 * length_J2_to_WC * link_2))
    theta2 = simplify(-pi/2 + angle_of_WC_from_J2 + internal_angle_of_J3_from_J2)


    internal_angle_of_WC_from_J3 = acos((link_2**2 + line_3_5**2 - length_J2_to_WC) / (2 * link_2 * line_3_5))
    theta3 = pi/2 + internal_angle_of_WC_from_J3 - angle_between_link_4_and_line_3_5

    

    print("theta2", theta2, "theta3", theta2)

    R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]

    R0_3 = R0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3.T * R_E    

    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
    print("theta4", theta4, "theta5", theta5, "theta6", theta6)

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    FK = T0_E.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    print(FK)

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx, wy, wz] # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3], FK[1,3], FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    #print ("\nTheta 1 error is: %04.8f" % t_1_e, theta1,test_case[2][0])
    print("\nTheta 1 error is: %04.8f (calculated: %s, expected: %04.8f)" % (t_1_e, theta1.evalf(),test_case[2][0]))
    print  ("Theta 2 error is: %04.8f (calculated: %s, expected: %04.8f)" % (t_2_e, theta2.evalf(),test_case[2][1]))
    print  ("Theta 3 error is: %04.8f (calculated: %s, expected: %04.8f)" % (t_3_e, theta3.evalf(),test_case[2][2]))
    print  ("Theta 4 error is: %04.8f" % t_4_e)
    print  ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 4

    test_code(test_cases[test_case_number])
