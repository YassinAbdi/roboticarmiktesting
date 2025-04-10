from Mathz import *
from browser import window
np= window.nj

class EEZYbotARM:
    """
    --Description--
    This is a parent class for the EExybot Robotic arm

    **Please note that you must always instantiate on a child class (either EEZYbotARMMk2() or EEZYbotARMMk1()) 
    rather than instansiating the parent 'EEZYbotARM' class**

    --Methods--
    Description of available methods in this class:    
        - __init__ --> Initialise a robotic arm with current joint position
        - updateJointAngles --> Update the stored joint angles for the robot arm
        - checkErrorJointLimits --> Check if any of the supplied joint angles are outside of physical limits
        - FK_EEZYbotARM --> Perform forward kinematics to find the end effector position, given joint angles
        - IK_EEZYbotARM --> Perform inverse kinematics to find the joint angles, given end effector position
        - plot_EEZYbotARM --> Plot the robotic arm in 3D using matplotlib Axes 3D

    """

    # Class attribute
    # e.g. species = 'mammal'

    # Initializer / Instance attributes
    def __init__(self, initial_q1, initial_q2, initial_q3):
        self.q1 = initial_q1
        self.q2 = initial_q2
        self.q3 = initial_q3

    # Instance methods
    def updateJointAngles(self, q1, q2, q3):
        """
        --Description--
        Update the three EEzybot Arm joint angles (by providing values in degrees)
        Will return an error message if the joint angles are outside of given limits

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees)
        @q2 -> the value of the angle q2 (in degrees) 
        @q3 -> the value of the angle q3 (in degrees)

        --Returns--
        Function doesn't return a value

        """
        # Check given values are inside valid limits
        self.checkErrorJointLimits(q1=q1, q2=q2, q3=q3)

        # Assign new angles
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def checkErrorJointLimits(self, **kwargs):
        """
        --Description--
        Take the three angles for the EzzyBot Arm defined in kinematics.
        Use these to calculate whether the EzzyBot Arm is outside of joint limits
        If outside of joint limts return error code for appropriate joint
        Otherwise return zero value

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        servoAngle_q1, servoAngle_q2, servoAngle_q3 -> values in degrees for output to the physical servos

        """
        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Find angle limits
        q3_min, q3_max = self.q3CalcLimits(q2=q2)

        # Check given angles are within limits, else raise exception
        if q1 < self.q1_min or q1 > self.q1_max:
            raise Exception('Value for q1 is outside joint limits. Joint limits in degrees are ({},{})'.format(
                self.q1_min, self.q1_max))

        if q2 < self.q2_min or q2 > self.q2_max:
            raise Exception('Value for q2 is outside joint limits. Joint limits in degrees are ({},{})'.format(
                self.q2_min, self.q2_max))

        if q3 < q3_min or q3 > q3_max:
            raise Exception(
                'Value for q3 is outside joint limits. Joint limits in degrees are ({},{})'.format(q3_min, q3_max))

    def forwardKinematics(self, **kwargs):
        """
        --Description--
        Given angles q1, q2, q3, find the forward kinematics for the end effector position of the EEZYbotARM

        If q1, q2, q3 are not provided then the current arm angles will be used (self.q1,q2,q3)

        The forward kinematics uses DH proximal convention and the method is disclosed seperate to this code

        The reference angle definitions are as follows:

        - EzzyBot base (q1) : 0 degree position is facing directly forwards
        - Main arm position (q2)
        - Horarm position(q3)

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        x_EE, y_EE, z_EE -> position of the end effector in the world frame (mm)

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Convert to radians
        q1 = q1 * pi/180
        q2 = q2 * pi/180
        q3 = q3 * pi/180

        # Joint length definitions (for ease of reading code)
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        # Find the position of the end effector using the forward kinematics equations
        x_EE = round((cos(q1) * (cos(q2+q3)*L3 + cos(q2)*L2))+(L4*cos(q1)), 3)
        y_EE = round((sin(q1) * (cos(q2+q3)*L3 + cos(q2)*L2))+(L4*sin(q1)), 3)
        z_EE = round((L1 + sin(q2)*L2 + sin(q2+q3)*L3), 3)

        # Return the end effector position in (mm)
        return x_EE, y_EE, z_EE

    def inverseKinematics(self, x_EE, y_EE, z_EE):
        """
        --Description--
        Function: to find the Inverse Kinematics for the EEZYbotARM Mk2 

        Description: given x y z positions of the desired EE position inside the
        manipulator workspace, find the corresponding joint angles

        --Parameters--
        x_EE, y_EE, z_EE -> the cartesian position of the end effector in the world frame

        --Returns--
        q1, q2, q3 -> Corresponding joint angles in degrees

        """
        # DH parameters (Proximal Convention)
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        # Find the value for the fist angle
        q1 = atan2(y_EE, x_EE)

        # Find the values for the position of joint #4 for x, y, z
        x_4 = x_EE - (L4 * cos(q1))
        y_4 = y_EE - (L4 * sin(q1))
        z_4 = z_EE

        # Find the length of the third side of the (virtual) triangle made by L2,
        # L3 in the vertical plane of the robot arm

        # --- Specify the z poisition of joint #1
        z_1 = L1

        # --- First we find the z distance between joint #1 and joint #4 (in the world frame)
        z_1_4 = z_4 - z_1

        # --- Find the true distance (in x, y plane) to joint #4
        xy_4 = sqrt((x_4**2)+(y_4**2))

        # --- Then we find the length of the virtual side made by the right angle triangle
        v_side = sqrt((z_1_4**2) + (xy_4**2))

        # Find the value for the angle at joint #3
        q3 = - (pi - acos((L2**2 + L3**2 - v_side**2)/(2 * L2 * L3)))

        # Find the value for the angle at joint #2 %DEBUG HERE
        # --- Find the value of the angle from the x-y plane to the virtual side
        q2_a = atan2(z_1_4, xy_4)

        q2_b = acos((v_side**2 + L2**2 - L3**2)/(2 * v_side * L2))

        q2 = q2_a + q2_b  # NOTE there's some more work to do here to make this correctly summation or subtraction dependant on the position of the arm!

        # # Print the input world frame position of the end effector
        # print('Input Position of End Effector: \n')
        # print('x_EE: {}'.format(x_EE))
        # print('y_EE: {}'.format(y_EE))
        # print('z_EE: {} \n'.format(z_EE))

        # # Print the output joint angles
        # print('Ouput joint angles: \n')
        # print('q1: {:+.2f}'.format(q1 * 180/pi))
        # print('q2: {:+.2f}'.format(q2 * 180/pi))
        # print('q3: {:+.2f} \n'.format(q3 * 180/pi))

        # round values
        q1 = round(q1 * 180/pi, 2)
        q2 = round(q2 * 180/pi, 2)
        q3 = round(q3 * 180/pi, 2)

        return q1, q2, q3
    
    def plot(self, **kwargs):
        """
        --Description--
        Given angles q1, q2, q3, plot EEZybot arm and co-ordinate frames for all joints using matplotlib
        Function computes the configuration of the manipulator and plots it using DH proximal convention.

        The function will raise an exception if any of the angles are outside defined limits

        The reference angle definitions are as follows:

        - EzzyBot base (q1) : 0 degree position is facing directly forwards
        - Main arm position (q2)
        - Horarm position(q3)

        --Required--
        Helper functions required (included in the EEzybot code)
        - plotCoOrd()
        - set_axes_radius()
        - set_axes_equal()

        --Optional **kwargs parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        fig, ax -> matplotlib figure and axes objects showing position of all actuator joints and links in world frame

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Convert angles to radians
        q1 = q1 * pi/180
        q2 = q2 * pi/180
        q3 = q3 * pi/180

        # DH parameters (Proximal Convention),
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4
        # Lengths for the links which attach to the hoarm
        L2A = self.L2A
        LAB = self.LAB
        LB3 = self.LB3

        # DH table
        DH = np.array([[0,  0,     L1, q1],
                       [0,  pi/2,  0,  q2],
                       [L2, 0,     0,  q3],
                       [L3, 0,     0,  0],
                       [0,  -pi/2, 0,  0]])

        # Find number of rows in DH table
        rows, cols = DH.shape

        # Pre-allocate Array to store Transformation matrix
        T = np.zeros((4, 4, rows), dtype=float)

        # Determine transformation matrix between each frame
        for i in range(rows):
            T[:, :, i] = [[cos(DH[i, 3]),              -sin(DH[i, 3]),               0,             DH[i, 0]],
                          [sin(DH[i, 3])*cos(DH[i, 1]),  cos(DH[i, 3]) *
                           cos(DH[i, 1]), -sin(DH[i, 1]), -sin(DH[i, 1])*DH[i, 2]],
                          [sin(DH[i, 3])*sin(DH[i, 1]),  cos(DH[i, 3]) *
                           sin(DH[i, 1]),  cos(DH[i, 1]),  cos(DH[i, 1])*DH[i, 2]],
                          [0,                          0,                          0,             1]]

        # Create the transformation frames with repect to the world frame (the base of the EEzybot arm)

        # --- Calculate Transformation matrix to each frame wrt the base. (matrix dot multiplication)
        T00 = np.identity(4)
        T01 = T[:, :, 0]
        T02 = T[:, :, 0].dot(T[:, :, 1])
        T03 = T[:, :, 0].dot(T[:, :, 1]).dot(T[:, :, 2])
        T04 = T[:, :, 0].dot(T[:, :, 1]).dot(T[:, :, 2]).dot(T[:, :, 3])

        # --- Create frame 5 (note this is just a rotation of frame T04)
        R5 = T04[0:3, 0:3]  # Find rotation matrix
        T45 = np.zeros((4, 4))
        T45[0:3, 0:3] = np.linalg.inv(R5)
        T45[3, 3] = 1  # Create transformation matrix from frame 4 to frame 5

        # Create the transformation matrix from the world frame to frame 5 (without z rotation)
        T05 = T04.dot(T45)

        # --- Construct a transformation matrix to make the Z rotation of T05 by magnitude q1
        TZRot = np.array([[cos(q1),  -sin(q1), 0, 0],
                          [sin(q1),   cos(q1), 0, 0],
                          [0,         0,       1, 0],
                          [0,         0,       0, 1]])

        # Create the transformation matrix from the world frame to frame 5 (with z rotation)
        T05_true = T05.dot(TZRot)

        # -- Create Frame EE (Do the same for the end effector frame)
        T5EE = np.array([[1, 0, 0, L4 * cos(q1)],
                         [0, 1, 0, L4 * sin(q1)],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

        TEE = T05.dot(T5EE).dot(TZRot)  # translate and rotate !

        # --- Create the frames for the links which attach to the hoarm
        q3_a = pi - (- q3)  # adjusted q3 value

        # --- --- For Frame A

        T2A_rot = np.array([[cos(q3_a),  -sin(q3_a), 0, 0],  # Rotation about z axis by q2
                            [sin(q3_a),   cos(q3_a), 0, 0],
                            [0,         0,       1, 0],
                            [0,         0,       0, 1]])

        T2A_trans = np.array([[1, 0, 0, L2A],  # Translation along x axis by distance L2A
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        T2A = T2A_rot.dot(T2A_trans)

        T0A = T02.dot(T2A)  # Frame for co-ordinate A

        # --- --- For Frame B

        TAB_rot = np.array([[cos(-(q3_a)),  -sin(-(q3_a)), 0, 0],  # Rotation about z axis by -(180-q2)
                            [sin(-(q3_a)),   cos(-(q3_a)), 0, 0],
                            [0,            0,          1, 0],
                            [0,            0,          0, 1]])

        TAB_trans = np.array([[1, 0, 0, LAB],  # Translation along x axis by distance L2A
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        TAB = TAB_rot.dot(TAB_trans)

        T0B = T0A.dot(TAB)  # Frame for co-ordinate A

        # Defines the position of each joint.
        # Joint 1 (This acts as the base)
        Base = np.array([0, 0, 0])
        J2_Pos = np.array([T01[0, 3], T01[1, 3], T01[2, 3]])  # Joint 2
        J3_Pos = np.array([T02[0, 3], T02[1, 3], T02[2, 3]])  # Joint 3
        # Joint 4. it is assumed that the origin of joint 4 and 5 coincide.
        J4_Pos = np.array([T03[0, 3], T03[1, 3], T03[2, 3]])
        J5_Pos = np.array([T04[0, 3], T04[1, 3], T04[2, 3]])
        EE_Pos = np.array([TEE[0, 3], TEE[1, 3], TEE[2, 3]])
        # --- Horarm joints
        A_Pos = np.array([T0A[0, 3], T0A[1, 3], T0A[2, 3]])
        B_Pos = np.array([T0B[0, 3], T0B[1, 3], T0B[2, 3]])

        # Coordinates for each link.
        Link1 = list(zip(Base, J2_Pos))   # From base to Joint 2
        Link2 = list(zip(J2_Pos, J3_Pos))  # From Joint 2 to Joint 3
        Link3 = list(zip(J3_Pos, J4_Pos))  # From Joint 3 to Joint 4
        Link4 = list(zip(J4_Pos, J5_Pos))  # From Joint 4 to Joint 5
        Link5 = list(zip(J5_Pos, EE_Pos))  # From Joint 5 to the end effector
        # --- Horarm links
        LinkA = list(zip(J2_Pos, A_Pos))  # From Joint 2 to Joint A
        LinkB = list(zip(A_Pos, B_Pos))  # From Joint A to Joint B
        LinkC = list(zip(B_Pos, J4_Pos))  # From Joint B to Joint 4

    #     # create the figure
    #     fig = plt.figure()

    #     # add an axis
    #     ax = fig.add_subplot(111, projection='3d')

    #     # add labels to the plot
    #     ax.set(title="3-d simulation of EEZYbotARM", xlabel="x (mm)",
    #            ylabel="y (mm)", zlabel="z (mm)")

    #     # set axis equal
    #     # ax.set_aspect('equal')         # important!

    #     # data for lines
    #     linewidth = 3

    #     # plot the lines
    #     ax.plot(Link1[0], Link1[1], Link1[2],
    #             label="Link1", linewidth=linewidth)
    #     #ax.plot(Link2[0],Link2[1], Link2[2], label="Link2", linewidth=linewidth)
    #     ax.plot(Link3[0], Link3[1], Link3[2],
    #             label="Link2", linewidth=linewidth)
    #     ax.plot(Link4[0], Link4[1], Link4[2],
    #             label="Link3", linewidth=linewidth)
    #     ax.plot(Link5[0], Link5[1], Link5[2],
    #             label="Link4", linewidth=linewidth)

    #     # --- plot the lines for the horarm links
    #     ax.plot(LinkA[0], LinkA[1], LinkA[2],
    #             linewidth=linewidth, color='lightgrey')
    #     ax.plot(LinkB[0], LinkB[1], LinkB[2],
    #             linewidth=linewidth, color='lightgrey')
    #     ax.plot(LinkC[0], LinkC[1], LinkC[2],
    #             linewidth=linewidth, color='lightgrey')

    #     # add a legend
    #     ax.legend()

    #     # plot co-ordinate frames
    #     plotCoOrd(T00, ax, lineColor='blue')
    #     plotCoOrd(T02, ax, lineColor='orange')
    #     plotCoOrd(T03, ax, lineColor='green')
    #     plotCoOrd(T05_true, ax, lineColor='red')
    #     plotCoOrd(TEE, ax, lineColor='grey')

    # #     plotCoOrd(T0A, ax, lineColor='lightgrey') # uncommenting will show servo 3 linkage co-ordinate frames
    # #     plotCoOrd(T0B, ax, lineColor='lightgrey')

    #     # show the plot
    #     # important code such that all axes display with equal scaling in 3d!
    #     set_axes_equal(ax)
    #     plt.show()

        # define the position of the end effector
        x_EE = round(float(TEE[0, 3]), 3)
        y_EE = round(float(TEE[1, 3]), 3)
        z_EE = round(float(TEE[2, 3]), 3)

        # Debug step
        print("plot_EEZYbotARM function --> End effector position (mm) is x: {}, y:{}, z:{}".format(x_EE, y_EE, z_EE))

        





# ------------------------------------------#
# Child class [inherits from EEZYbotARM() class]
# ------------------------------------------#

class EEZYbotARM_Mk2(EEZYbotARM):
    """
    --Description--
    This is a child class for the EEzybot Robotic arm MK2 (inherits from EEZYbotARM() class) 

    **Please note that you must always instantiate on a child class (either EEZYbotARMMk2() or EEZYbotARMMk1()) 
    rather than instansiating the parent 'EEZYbotARM' class**

    --Methods--
    Description of available methods in this class:    
        - q3CalcLimits --> Calculate the physical limits for joint 3 (because these depend on the angle of joint 2)
        - map_kinematicsToServoAngles --> Map angles defined in kinematics to physical servo angles on the robot
    """

    # Class attribute
    # DH parameters (Proximal Convention)
    L1 = 92  # mm
    L2 = 135
    L3 = 147
    L4 = 87

    # --- Lengths of links which attach to the hoarm
    L2A = 60
    LAB = 140
    LB3 = 60

    # Joint limits
    q1_min = -30  # degrees
    q1_max = 30
    q2_min = 39
    q2_max = 120
    # q3_min, q3_max are given by q3CalcLimits() function

    def q3CalcLimits(self, **kwargs):
        """
        Calculate q3 angle limits for the EzzyBot Arm given a value for the angle q2 in degrees
        These limits have been determined experimentally for the EEzybotMk2

        If no q2 value is given then the current value of q2 is used 

        --Optional kwargs Parameters--
        @q2 -> the value of the angle q2 (in degrees)

        --Returns--
        q3_min, q3_max -> the min and max limits for the angle q3 (in degrees)

        """
        # Use **kwarg if provided, otherwise use current q2 value
        q2 = kwargs.get('q2', self.q2)

        # calculate q3 min limits in degrees
        q3_min = (-0.6755 * q2) - 70.768
        q3_max = (-0.7165 * q2) - 13.144

        return q3_min, q3_max

    def map_kinematicsToServoAngles(self, **kwargs):
        """
        --Description--
        Take the three angles for the EzzyBot Arm defined in kinematics.
        Use these to calculate the required physical servo position with respect to the reference position.
        If three angles are not provided as **kwargs then the current values for the arm are used

        The reference positions for the three servos are as follows:

        EzzyBot base (q1) : 90 degree servo position is facing directly forwards
        Main arm (q2): 90 degree servo position is with main arm perpendicular (at 90 degrees to) base
        Horarm (q3): 90 degree servo poisition is with horarm servo link at 45 degrees to base

        The function will be updated to raise an error message when any of the returned angles are outside of the servo limits.

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        servoAngle_q1, servoAngle_q2, servoAngle_q3 -> values in degrees for output to the physical servos

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Check none of the angles are outside of joint limits! So that servos cannot get damaged
        self.checkErrorJointLimits(q1=q1, q2=q2, q3=q3)

        # Calculate for q1
        servoAngle_q1 = ((-2.0497)*q1) + 91.726  # from experimentation !
        servoAngle_q1 = round(servoAngle_q1, 2)

        # Calculate for q2
        servoAngle_q2 = 180 - q2  # approximate adjusted q2 value
        servoAngle_q2 = round(servoAngle_q2, 2)

        # Calculate for q3
        q3_a = 180 - (- q3)  # approximate adjusted q3 value
        servoAngle_q3 = q2 - 45 + q3_a
        servoAngle_q3 = round(servoAngle_q3, 2)

        return servoAngle_q1, servoAngle_q2, servoAngle_q3