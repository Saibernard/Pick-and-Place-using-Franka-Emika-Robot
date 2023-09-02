import numpy as np
import math
from math import pi

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here


        alpha = [0, -pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, -pi / 2, 0]
        d = [0.141, 0.192, 0, 0.316, 0, 0.384, 0, 0.210]
        a = [0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0]

        t1_w = np.array([[np.cos(0), -np.sin(0) * (np.cos(alpha[0])), np.sin(0) * (np.sin(alpha[0])),
                          a[0] * (np.cos(0))], [np.sin(0), np.cos(0) * (np.cos(alpha[0])),
                                                   -np.cos(0) * (np.sin(alpha[0])), a[0] * (np.sin(0))],
                         [0, np.sin(alpha[0]), np.cos(alpha[0]), d[0]], [0, 0, 0, 1]])
        t2_1 = np.array([[np.cos(q[0]), -np.sin(q[0]) * (np.cos(alpha[1])), np.sin(q[0]) * (np.sin(alpha[1])),
                          a[1] * (np.cos(q[0]))], [np.sin(q[0]), np.cos(q[0]) * (np.cos(alpha[1])),
                                                   -np.cos(q[0]) * (np.sin(alpha[1])), a[1] * (np.sin(q[0]))],
                         [0, np.sin(alpha[1]), np.cos(alpha[1]), d[1]], [0, 0, 0, 1]])
        t3_2 = np.array([[np.cos(q[1]), -np.sin(q[1]) * (np.cos(alpha[2])), np.sin(q[1]) * (np.sin(alpha[2])),
                          a[2] * (np.cos(q[1]))], [np.sin(q[1]), np.cos(q[1]) * (np.cos(alpha[2])),
                                                   -np.cos(q[1]) * (np.sin(alpha[2])), a[2] * (np.sin(q[1]))],
                         [0, np.sin(alpha[2]), np.cos(alpha[2]), d[2]], [0, 0, 0, 1]])
        t4_3 = np.array([[np.cos(q[2]), -np.sin(q[2]) * (np.cos(alpha[3])), np.sin(q[2]) * (np.sin(alpha[3])),
                          a[3] * (np.cos(q[2]))], [np.sin(q[2]), np.cos(q[2]) * (np.cos(alpha[3])),
                                                   -np.cos(q[2]) * (np.sin(alpha[3])), a[3] * (np.sin(q[2]))],
                         [0, np.sin(alpha[3]), np.cos(alpha[3]), d[3]], [0, 0, 0, 1]])
        t5_4 = np.array([[np.cos(q[3]-pi), -np.sin(q[3]-pi) * (np.cos(alpha[4])), np.sin(q[3]-pi) * (np.sin(alpha[4])),
                          a[4] * (np.cos(q[3]-pi))], [np.sin(q[3]-pi), np.cos(q[3]-pi) * (np.cos(alpha[4])),
                                                    -np.cos(q[3]-pi) * (np.sin(alpha[4])), a[4] * (np.sin(q[3]-pi))],
                         [0, np.sin(alpha[4]), np.cos(alpha[4]), d[4]], [0, 0, 0, 1]])
        t6_5 = np.array([[np.cos(q[4]), -np.sin(q[4]) * (np.cos(alpha[5])), np.sin(q[4]) * (np.sin(alpha[5])),
                          a[5] * (np.cos(q[4]))], [np.sin(q[4]), np.cos(q[4]) * (np.cos(alpha[5])),
                                                   -np.cos(q[4]) * (np.sin(alpha[5])), a[5] * (np.sin(q[4]))],
                         [0, np.sin(alpha[5]), np.cos(alpha[5]), d[5]], [0, 0, 0, 1]])
        t7_6 = np.array([[np.cos(pi-q[5]), -np.sin(pi-q[5]) * (np.cos(alpha[6])), np.sin(pi-q[5]) * (np.sin(alpha[6])),
                          a[6] * (np.cos(pi-q[5]))], [np.sin(pi-q[5]), np.cos(pi-q[5]) * (np.cos(alpha[6])),
                                                   -np.cos(pi-q[5]) * (np.sin(alpha[6])), a[6] * (np.sin(pi-q[5]))],
                         [0, np.sin(alpha[6]), np.cos(alpha[6]), d[6]], [0, 0, 0, 1]])
        te_7 = np.array([[np.cos(q[6]-(pi/4)), -np.sin(q[6]-(pi/4)) * (np.cos(alpha[7])), np.sin(q[6]-(pi/4)) * (np.sin(alpha[7])),
                          a[7] * (np.cos(q[6]-(pi/4)))], [np.sin(q[6]-(pi/4)), np.cos(q[6]-(pi/4)) * (np.cos(alpha[7])),
                                                        -np.cos(q[6]-(pi/4)) * (np.sin(alpha[7])),
                                                        a[7] * (np.sin(q[6]-(pi/4)))],
                         [0, np.sin(alpha[7]), np.cos(alpha[7]), d[7]], [0, 0, 0, 1]])
        joint_1_transformation = t1_w
        joint_2_transformation = joint_1_transformation @ t2_1
        joint_3_transformation = joint_2_transformation @ t3_2
        joint_4_transformation = joint_3_transformation @ t4_3
        joint_5_transformation = joint_4_transformation @ t5_4
        joint_6_transformation = joint_5_transformation @ t6_5
        joint_7_transformation = joint_6_transformation @ t7_6
        joint_e_transformation = joint_7_transformation @ te_7

        RM_1 = joint_1_transformation[:-1, :-1]
        RM_2 = joint_2_transformation[:-1, :-1]
        RM_3 = joint_3_transformation[:-1, :-1]
        RM_4 = joint_4_transformation[:-1, :-1]
        RM_5 = joint_5_transformation[:-1, :-1]
        RM_6 = joint_6_transformation[:-1, :-1]
        RM_7 = joint_7_transformation[:-1, :-1]
        RM_e = joint_e_transformation[:-1, :-1]

        TM_1 = joint_1_transformation[:-1, -1:]
        TM_2 = joint_2_transformation[:-1, -1:]
        # TM_3 = joint_3_transformation[:-1, -1:]
        TM_4 = joint_4_transformation[:-1, -1:]
        # TM_5 = joint_5_transformation[:-1, -1:]
        # TM_6 = joint_6_transformation[:-1, -1:]
        # TM_7 = joint_7_transformation[:-1, -1:]
        TM_e = joint_e_transformation[:-1, -1:]

        third_joint_position = np.array([[0], [0], [0.195], [1]])
        thirdjoint_pos = joint_3_transformation @ third_joint_position
        TM_3 = thirdjoint_pos[:-1, -1:]

        fifth_joint_position = np.array([[0], [0], [0.125], [1]])
        fifthjoint_pos = joint_5_transformation @ fifth_joint_position
        TM_5 = fifthjoint_pos[:-1, -1:]
        sixth_joint_position = np.array([[0], [0], [0.015], [1]])
        sixthjoint_pos = joint_6_transformation @ sixth_joint_position
        TM_6 = sixthjoint_pos[:-1, -1:]
        seventh_joint_position = np.array([[0], [0], [0.051], [1]])
        seventhjoint_pos = joint_7_transformation @ seventh_joint_position
        TM_7 = seventhjoint_pos[:-1, -1:]
        np.set_printoptions(formatter={'float_kind': '{:.2f}'.format})

        # print("The position of 1st joint with respect to base frame: \n", TM_1)
        # print("The position of 2nd joint with respect to base frame:\n", TM_2)
        # print("The position of 3rd joint with respect to base frame:\n", TM_3)
        # print("The position of 4th joint with respect to base frame:\n", TM_4)
        # print("The position of 5th joint with respect to base frame:\n", TM_5)
        # print("The position of 6th joint with respect to base frame:\n", TM_6)
        # print("The position of 7th joint with respect to base frame:\n", TM_7)
        # print("The position of end effector joint with respect to base frame\n:", TM_e)

        TM_1T = np.transpose(TM_1)

        TM_2T = np.transpose(TM_2)

        TM_3T = np.transpose(TM_3)

        TM_4T = np.transpose(TM_4)

        TM_5T = np.transpose(TM_5)

        TM_6T = np.transpose(TM_6)

        TM_7T = np.transpose(TM_7)

        TM_eT = np.transpose(TM_e)

        # jointPositions = np.array([[TM_1],[TM_2],[TM_3],[TM_4],[TM_5],[TM_6],[TM_7],[TM_e]])
        jointPositions = np.vstack([TM_1T, TM_2T, TM_3T, TM_4T, TM_5T, TM_6T, TM_7T, TM_eT])
        T0e = joint_e_transformation

        # Your code ends here

        return jointPositions, T0e


        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1


    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()

    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()

if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    # q = np.array([0,0,0,-pi/2,0,pi/2,0])
    q = np.array([-1.28, 0.03, 1.46, -2.29, 0.00, 2.27, 0.94])

    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
