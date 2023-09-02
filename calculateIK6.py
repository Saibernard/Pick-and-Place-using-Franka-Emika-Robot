import numpy as np
from math import pi
from lib.calculateFK import FK

class IK:
    """
    Solves the 6 DOF (joint 5 fixed) IK problem for panda robot arm
    """
    # offsets along x direction
    a1 = 0
    a2 = 0
    a3 = 0.0825
    a4 = 0.0825
    a5 = 0
    a6 = 0.088
    a7 = 0

    # offsets along z direction
    d1 = 0.333
    d2 = 0
    d3 = 0.316
    d4 = 0
    d5 = 0.384
    d6 = 0
    d7 = 0.210

    # This variable is used to express an arbitrary joint angle
    Q0 = 0.123

    # Used to store transformation matrices
    T07 = []
    T70 = []
    T06 = []
    T60 = []

    # Joint limits
    upper_lims = [2.8973,   1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]
    lower_lims = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]

    def build_matrix(self, a, d, alpha, theta):
        m = np.zeros((4,4))
        m[0][0] = np.cos(theta)
        m[0][1] = -np.sin(theta) * np.cos(alpha)
        m[0][2] = np.sin(theta) * np.sin(alpha)
        m[0][3] = a * np.cos(theta)

        m[1][0] = np.sin(theta)
        m[1][1] = np.cos(theta) * np.cos(alpha)
        m[1][2] = -np.cos(theta) * np.sin(alpha)
        m[1][3] = a * np.sin(theta)

        m[2][0] = 0
        m[2][1] = np.sin(alpha)
        m[2][2] = np.cos(alpha)
        m[2][3] = d

        m[3][0] = 0
        m[3][1] = 0
        m[3][2] = 0
        m[3][3] = 1

        return m

    def angleRound(self, ang):
        if (ang < -np.pi or ang > np.pi):
            trunc_pos = np.sign(ang) * ((np.abs(ang) % (2*np.pi)))
            if (trunc_pos > np.pi):
                return trunc_pos - 2*np.pi
            elif (trunc_pos < -np.pi):
                return trunc_pos + 2*np.pi
            else:
                return trunc_pos
        else:
            return ang

    def invertT(self, R, t):
        """
        Creates an inverted T matrix from the given R and t.
        Args:
            R: Rotation matrix.
            t: Translation vector.
        """
        Tinv_R = R.T
        Tinv_R = np.append(Tinv_R, [np.array([0, 0, 0])], axis=0)
        Tinv_t = np.append(-R.T @ t, [1]).reshape((4,1))
        Tinv = np.append(Tinv_R, Tinv_t, axis=1)
        return Tinv

    def panda_ik(self, target):
        """
        Solves 6 DOF IK problem given physical target in x, y, z space
        Args:
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base
                't': numpy array of the end effector position relative to the robot base

        Returns:
             q = nx7 numpy array of joints in radians (q5: joint 5 angle should be 0)
        """

        # Extract the R and t matrices
        R = target['R']
        t = target['t']

        # Initialize q
        q = np.zeros((0,7))

        # Student's code goes in between:

        #Find transformation matrix
        T_R = np.append(R, [np.array([0, 0, 0])], axis=0)
        T_t = np.append(t, [1]).reshape((4,1))
        self.T07 = np.append(T_R, T_t, axis=1)

        # Find inverse transformation matrix
        self.T70 = self.invertT(target['R'], target['t'])
        wrist_pos = self.kin_decouple(target)
        joints_467 = self.ik_pos(wrist_pos)
        joints_123 = self.ik_orient(R, joints_467)

        # Compile solutions
        for i in range(4):
            for j in range(2):
                q = np.append(q, [joints_123[i*2+j][0], joints_123[i*2+j][1], joints_123[i*2+j][2], joints_467[i][0], 0, joints_467[i][1], joints_467[i][2]])
        q = q.reshape((8,7))

        q_cleaned = np.empty((0,7))
        rows = 0
        for q_i in q:
            if (not(np.any(np.isnan(q_i))) and np.all(q_i <= self.upper_lims) and np.all(q_i >= self.lower_lims)):
                rows+=1
                q_cleaned = np.append(q_cleaned, q_i)
        print("q!!!")
        print(q)
        q = q_cleaned.reshape((rows, 7))
        print(q)
        if (q.shape[0] == 0):
            return q
        q = np.unique(q, axis=0)
        print(q)
        # Student's code goes in between:

        ## DO NOT EDIT THIS PART
        # This will convert your joints output to the autograder format
        q = self.sort_joints(q)
        ## DO NOT EDIT THIS PART
        return q

    def kin_decouple(self, target):
        """
        Performs kinematic decoupling on the panda arm to find the position of wrist center
        Args:
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base
                't': numpy array of the end effector position relative to the robot base

        Returns:
             wrist_pos = 3x1 numpy array of the position of the wrist center in frame 7
        """
        R = target['R']
        t = target['t']

        T70 = self.invertT(R, t)

        o70 = (T70 @ np.array([0, 0, 0, 1]))[0:3]

        wrist_pos = o70 - (-self.d1) * T70[0:3, 2]
        return wrist_pos

    def ik_pos(self, wrist_pos):
        """
        Solves IK position problem on the joint 4, 6, 7
        Args:
            wrist_pos: 3x1 numpy array of the position of the wrist center in frame 7

        Returns:
             joints_467 = nx3 numpy array of all joint angles of joint 4, 6, 7
        """

        x = 0
        y = 1
        z = 2

        joints_467 = []
        o72 = wrist_pos
        # Find q7
        if (not(np.round(o72[x], 5) == 0 and np.round(o72[y], 5) == 0)):
            q7 = [-self.angleRound(np.pi - (np.arctan2(-o72[y], o72[x]) + np.pi/4))]
            q7 = np.append(q7, [self.angleRound(q7[0] - np.pi)])
        else:
            q7 = np.array([self.Q0, self.Q0 - np.pi])

        # Find q4 and q6
        q4 = []
        q6 = []
        for q7_i in q7:
            # Find T67 transformation
            H7 = self.build_matrix(0,self.d7,0,q7_i - np.pi/4)
            # self.T06 = self.T07 @ self.invertT(H7[0:3,0:3], H7[:3,3])
            # self.T60 = self.invertT(self.T06[0:3,0:3], self.T06[:3,3])
            # self.T67 = self.T60 @ self.T07

            # Get relevant points in frame 6
            o65 = np.array([-self.a6, 0, 0, 1])
            o62 = H7 @ np.append(o72, [1])

            # Get triangle sides
            A = np.sqrt(np.power(self.d3,2) + np.power(self.a3, 2))
            B = np.sqrt(np.power(self.d5,2) + np.power(self.a3, 2))
            C = np.linalg.norm(o62-o65)

            # Calculate angles
            theta2 = np.arccos((np.power(C, 2) - np.power(A,2) - np.power(B,2)) / (2 * A * B))

            q4_a = self.angleRound(theta2 + np.arctan(self.d3/self.a3) + np.arctan(self.d5/self.a3) - np.pi)
            q4_acomp = self.angleRound(-theta2 + np.arctan(self.d3/self.a3) + np.arctan(self.d5/self.a3) - np.pi)

            q4_generated = np.array([q4_a, q4_acomp])

            q6_generated = []
            for theta2_i in [theta2, -theta2]:
                # thetaOffset = np.arctan(self.d3/self.a3) + np.arctan(self.d5/self.a3)
                # thetaOffset = np.arcsin([(A * (np.sin(thetaOffset) / C))])
                # theta1Comp = (np.abs(np.arctan([np.linalg.norm(o62[z]-o65[z])/np.linalg.norm(o62[x]-o65[x])])) + np.abs(np.arcsin([(A * (np.sin(theta2_i) / C))]) - thetaOffset))
                # q6 = np.append(q6, [self.angleRound(np.pi - theta1Comp - np.pi/2)])
                # print(theta2_i)
                # print(np.arctan(self.a3/self.d5))
                # print(np.arctan2(A*np.sin(theta2_i), B+A*np.cos(theta2_i)))
                o2o5Angle = np.arctan2(o62[z]-o65[z], o62[x]-o65[x])
                # print((np.sign(o62[z]-o65[z])*np.linalg.norm(o62[z]-o65[z])))
                # print((np.sign(o62[x]-o65[x])*np.linalg.norm(o62[x]-o65[x])))
                # print(o2o5Angle)
                q6_i = self.angleRound(((o2o5Angle + np.arctan(self.a3/self.d5) - np.arctan2(A*np.sin(theta2_i), B+A*np.cos(theta2_i)) - np.pi/2)))
                q6_generated = np.append(q6_generated, [q6_i])

            joints_467 = np.append(joints_467, np.array([q4_generated[0], q6_generated[0], q7_i]), axis=0)
            joints_467 = np.append(joints_467, np.array([q4_generated[1], q6_generated[1], q7_i]), axis=0)
        # print(joints_467.reshape((4,3)))
        return joints_467.reshape((4,3))

    def ik_orient(self, R, joints_467):
        """
        Solves IK orientation problem on the joint 1, 2, 3
        Args:
            R: numpy array of the end effector pose relative to the robot base
            joints_467: nx3 numpy array of all joint angles of joint 4, 6, 7

        Returns:
            joints_123 = nx3 numpy array of all joint angles of joint 1, 2 ,3
        """
        import pdb
        joints_123 = []
        j4 = 0
        j6 = 1
        j7 = 2
        for j_467 in joints_467:
            H67 = self.build_matrix(0,self.d7,0,j_467[j7] - np.pi/4)
            H56 = self.build_matrix(self.a6,0,np.pi/2,j_467[j6])
            H45 = self.build_matrix(0,self.d5,np.pi/2,0)
            H34 = self.build_matrix(-self.a4,0,-np.pi/2,j_467[j4])

            R34 = H34[0:3,0:3]
            R45 = H45[0:3,0:3]
            R56 = H56[0:3,0:3]
            R67 = H67[0:3,0:3]

            R03 = R @ (R34 @ R45 @ R56 @ R67).T

            theta = 0
            theta_neg = 0
            phi = 0
            phi_neg = 0
            psi = 0
            psi_neg = 0

            # Find theta
            q2 = (np.arccos(R03[2,1]))
            q2_neg = -q2

            # Find phi
            cos_q1 = np.round(R03[0,1]/np.sin(q2), 12)
            sin_q1 = np.round(R03[1,1]/np.sin(q2), 12)
            q1 = np.round(np.arctan2(sin_q1, cos_q1), 12)

            cos_q1_neg = np.round(R03[0,1]/np.sin(q2_neg), 12)
            sin_q1_neg = np.round(R03[1,1]/np.sin(q2_neg), 12)
            q1_neg = np.round(np.arctan2(sin_q1_neg, cos_q1_neg), 12)

            # Find psi
            cos_q3 = -np.round(R03[2,0]/np.sin(q2), 12)
            sin_q3 = -np.round(R03[2,2]/np.sin(q2), 12)
            q3 = np.round(np.arctan2(sin_q3, cos_q3), 12)


            cos_q3_neg = -np.round(R03[2,0]/np.sin(q2_neg), 12)
            sin_q3_neg = -np.round(R03[2,2]/np.sin(q2_neg), 12)
            q3_neg = np.round(np.arctan2(sin_q3_neg, cos_q3_neg), 12)

            if (np.round(q2, 6) == 0):
                # Find phi
                q1 = -self.Q0
                q1_neg = -self.Q0

                # Find psi
                q3 = self.Q0
                q3_neg = self.Q0

            # Map euler angles
            j3 = self.angleRound(q3)
            j3_neg = self.angleRound(q3_neg)
            j2 = self.angleRound(q2)
            j2_neg = self.angleRound(q2_neg)
            j1 = self.angleRound(q1)
            j1_neg = self.angleRound(q1_neg)

            joints_123 = np.append(joints_123, np.array([j1, j2, j3]))
            joints_123 = np.append(joints_123, np.array([j1_neg, j2_neg, j3_neg]))

        joints_123 = joints_123.reshape((8,3))

        return joints_123



    def sort_joints(self, q, col=0):
        """
        Sort the joint angle matrix by ascending order
        Args:
            q: nx7 joint angle matrix
        Returns:
            q_as = nx7 joint angle matrix in ascending order
        """
        if col != 7:
            q_as = q[q[:, col].argsort()]
            for i in range(q_as.shape[0]-1):
                if (q_as[i, col] < q_as[i+1, col]):
                    # do nothing
                    pass
                else:
                    for j in range(i+1, q_as.shape[0]):
                        if q_as[i, col] < q_as[j, col]:
                            idx = j
                            break
                        elif j == q_as.shape[0]-1:
                            idx = q_as.shape[0]

                    q_as_part = self.sort_joints(q_as[i:idx, :], col+1)
                    q_as[i:idx, :] = q_as_part
        else:
            q_as = q[q[:, -1].argsort()]
        return q_as

def main():

    # fk solution code
    fk = FK()

    # input joints
    q1 = -1.76
    q2 = 1.325
    q3 = -0.167
    q4 = -0.679
    q5 = 1.52
    q6 = 1.62
    q7 = -1.18

    q_in  = np.array([q1, q2, q3, q4, q5, q6, q7])
    [_, T_fk] = fk.forward(q_in)

    # input of IK class
    target = {'R': T_fk[0:3, 0:3], 't': T_fk[0:3, 3]}
    ik = IK()
    print('target', target)
    # target
    q = ik.panda_ik(target)
    print("Angles",q.T)
    # verify IK solutions
    for i in range(q.shape[0]):
        [_, T_ik] = fk.forward(q[i, :])
        print('Matrix difference = ')
        print(np.round(T_fk - T_ik, 1))
        print()

if __name__ == '__main__':
    main()
